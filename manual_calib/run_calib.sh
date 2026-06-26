#!/usr/bin/env bash
# Interactive launcher for manual lidar2camera calibration.
# Usage: ./run_calib.sh   (run from manual_calib/ or any directory)

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DATA_DIR="${SCRIPT_DIR}/data"
BIN="${SCRIPT_DIR}/bin/run_lidar2camera"

select_from_list() {
    local prompt="$1"
    shift
    local -a items=("$@")
    local i choice

    if [[ ${#items[@]} -eq 0 ]]; then
        echo "Error: no options available for: ${prompt}" >&2
        exit 1
    fi

    if [[ ${#items[@]} -eq 1 ]]; then
        echo "Auto-selected (only option): ${items[0]}" >&2
        echo "${items[0]}"
        return
    fi

    echo "" >&2
    echo "${prompt}" >&2
    for i in "${!items[@]}"; do
        printf "  [%d] %s\n" "$((i + 1))" "${items[$i]}" >&2
    done

    while true; do
        read -r -p "Enter number (1-${#items[@]}): " choice
        if [[ "$choice" =~ ^[0-9]+$ ]] && (( choice >= 1 && choice <= ${#items[@]} )); then
            echo "${items[$((choice - 1))]}"
            return
        fi
        echo "Invalid input, please try again." >&2
    done
}

pick_file() {
    local label="$1"
    local dir="$2"
    shift 2
    local -a patterns=("$@")
    local -a files=()
    local pat f

    for pat in "${patterns[@]}"; do
        while IFS= read -r -d '' f; do
            files+=("$f")
        done < <(find "$dir" -maxdepth 1 -type f -name "$pat" -print0 2>/dev/null | sort -z)
    done

    # Deduplicate while preserving order
    local -a unique=()
    local seen item
    for item in "${files[@]}"; do
        seen=0
        for f in "${unique[@]:-}"; do
            if [[ "$f" == "$item" ]]; then
                seen=1
                break
            fi
        done
        if [[ $seen -eq 0 ]]; then
            unique+=("$item")
        fi
    done

    if [[ ${#unique[@]} -eq 0 ]]; then
        echo "Error: no ${label} found in ${dir}" >&2
        exit 1
    fi

    if [[ ${#unique[@]} -eq 1 ]]; then
        basename "${unique[0]}"
        return
    fi

    local -a names=()
    for f in "${unique[@]}"; do
        names+=("$(basename "$f")")
    done
    select_from_list "Multiple ${label} files found, please choose:" "${names[@]}"
}

main() {
    if [[ ! -x "$BIN" ]]; then
        echo "Error: executable not found: ${BIN}" >&2
        echo "Please build first: mkdir -p build && cd build && cmake .. && make" >&2
        exit 1
    fi

    if [[ ! -d "$DATA_DIR" ]]; then
        echo "Error: data directory not found: ${DATA_DIR}" >&2
        exit 1
    fi

    # Collect vehicle datasets: subdirs of data/ that contain camera subfolders
    local -a vehicles=()
    local d sub has_camera
    for d in "$DATA_DIR"/*/; do
        [[ -d "$d" ]] || continue
        has_camera=0
        for sub in "$d"*/; do
            [[ -d "$sub" ]] || continue
            if compgen -G "${sub}"*-intrinsic.json >/dev/null 2>&1; then
                has_camera=1
                break
            fi
        done
        if [[ $has_camera -eq 1 ]]; then
            vehicles+=("$(basename "$d")")
        fi
    done

    if [[ ${#vehicles[@]} -eq 0 ]]; then
        echo "Error: no vehicle datasets found under ${DATA_DIR}" >&2
        exit 1
    fi

    local vehicle
    vehicle="$(select_from_list "Select vehicle dataset:" "${vehicles[@]}")"
    local vehicle_dir="${DATA_DIR}/${vehicle}"

    # Collect camera folders
    local -a cameras=()
    local cam_dir
    for cam_dir in "$vehicle_dir"/*/; do
        [[ -d "$cam_dir" ]] || continue
        if compgen -G "${cam_dir}"*-intrinsic.json >/dev/null 2>&1; then
            cameras+=("$(basename "$cam_dir")")
        fi
    done

    if [[ ${#cameras[@]} -eq 0 ]]; then
        echo "Error: no camera folders found in ${vehicle_dir}" >&2
        exit 1
    fi

    local camera
    camera="$(select_from_list "Select camera for vehicle ${vehicle}:" "${cameras[@]}")"
    local cam_path="${vehicle_dir}/${camera}"

    local intrinsic="${camera}-intrinsic.json"
    local extrinsic="top_center_lidar-to-${camera}-extrinsic.json"

    if [[ ! -f "${cam_path}/${intrinsic}" ]]; then
        echo "Error: intrinsic file not found: ${cam_path}/${intrinsic}" >&2
        exit 1
    fi
    if [[ ! -f "${cam_path}/${extrinsic}" ]]; then
        echo "Error: extrinsic file not found: ${cam_path}/${extrinsic}" >&2
        exit 1
    fi

    local image pcd
    image="$(pick_file "image (.jpg)" "$cam_path" "${camera}_*.jpg" "*.jpg")"
    pcd="$(pick_file "point cloud (.pcd)" "$cam_path" "*.pcd")"

    local rel_base="data/${vehicle}/${camera}"
    local cmd=(
        "$BIN"
        "${rel_base}/${image}"
        "${rel_base}/${pcd}"
        "${rel_base}/${intrinsic}"
        "${rel_base}/${extrinsic}"
    )

    echo ""
    echo "Vehicle : ${vehicle}"
    echo "Camera  : ${camera}"
    echo "Command :"
    printf '  %q' "${cmd[@]}"
    echo ""
    echo ""

    cd "$SCRIPT_DIR"
    exec "${cmd[@]}"
}

main "$@"
