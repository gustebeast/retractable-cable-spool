"""Retractable Cable Spool — cadquery model split into focused modules.

  dimensions  — top-level dimensional/material/fit constants
  helpers     — geometric helper functions (cyl, flange, ratchet, ...)
  build       — main script: constructs every part and writes STEP files

Run from the repo root:
  py -3.12 -m src.build
"""
