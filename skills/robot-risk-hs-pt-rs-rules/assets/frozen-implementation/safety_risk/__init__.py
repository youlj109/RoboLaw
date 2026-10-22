# pylint: skip-file
"""Safety risk evaluation pipeline for simulation benchmarks.

This module provides a rule-based safety risk evaluation pipeline that processes
simulation episodes from raw GT signals through feature extraction to risk level
assessment (HS/PT/RS/IR L0-L3).

Derived from the data contract in robot_safety_risk_data_contract.xlsx.
"""

__version__ = "0.1.0"
