#!/usr/bin/env python3

from pathlib import Path


def clear_debug_csv() -> None:
    debug_dir = Path(__file__).resolve().parent
    csv_dir = debug_dir / "csv"

    if not csv_dir.exists():
        print(f"CSV folder not found: {csv_dir}")
        return

    csv_files = sorted(p for p in csv_dir.glob("*.csv") if p.is_file())

    if not csv_files:
        print(f"No CSV files found in {csv_dir}")
        return

    for csv_file in csv_files:
        csv_file.unlink()
        print(f"Deleted {csv_file.name}")


if __name__ == "__main__":
    clear_debug_csv()
