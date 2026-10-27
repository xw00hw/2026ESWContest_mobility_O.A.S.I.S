"""Part A (OBD): reproduce all outputs from 00_raw.

  python pipeline/1_vehicle_obd_ocr/run.py            # reuse caches in _work/1_vehicle_obd_ocr/cache when present
  python pipeline/1_vehicle_obd_ocr/run.py --force    # rebuild everything from the raw video (approx. 15 min)

Outputs: 01_refined/run1/obd.csv, 01_refined/run2/obd.csv (+ obd_text.csv, obd_changes.csv),
         02_qc/1_vehicle_obd_ocr/{run1_checks.json, run2_stats.json, run2_selfcheck.json, plots/}
templates.npz (glyph templates of the Infocar font) is a pipeline asset; it was built once with
build_templates.py (clusters labelled by eye) and is not regenerated here.
"""
import sys

import run1_xlsx
import stage1_extract
import stage2_ocr
import stage3_assemble
import stage4_selfcheck
from common import CACHE

if __name__ == "__main__":  # guard needed: stage 2 uses multiprocessing (spawn on Windows)
    force = "--force" in sys.argv
    run1_xlsx.main()
    if force or not (CACHE / "stage1.pkl").exists():
        stage1_extract.main()
    if force or not (CACHE / "stage2.pkl").exists():
        stage2_ocr.main()
    stage3_assemble.main()
    stage4_selfcheck.main()
    print("Part A done")
