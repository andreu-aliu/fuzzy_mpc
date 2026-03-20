#!/usr/bin/env python3

from pathlib import Path


def clear_debug_csv() -> None:
    debug_dir = Path(__file__).resolve().parent
    csv_files = sorted(debug_dir.glob("*.csv"))

    if not csv_files:
        print(f"No CSV files found in {debug_dir}")
        return

    for csv_file in csv_files:
        csv_file.unlink()
        print(f"Deleted {csv_file.name}")


if __name__ == "__main__":
    clear_debug_csv()
