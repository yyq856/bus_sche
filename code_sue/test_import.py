#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""Quick syntax check for solve_integrated_sue.py"""
import sys
from pathlib import Path

code_sue_path = Path(__file__).resolve().parent
if code_sue_path not in sys.path:
    sys.path.insert(0, str(code_sue_path))

try:
    import solve_integrated_sue
    print("✓ solve_integrated_sue.py imports successfully")
    print(f"✓ Found functions: {[x for x in dir(solve_integrated_sue) if not x.startswith('_')]}")
except Exception as e:
    print(f"✗ Import failed: {e}")
    import traceback
    traceback.print_exc()
