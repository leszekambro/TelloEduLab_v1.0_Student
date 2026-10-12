@echo off
python -m pip install --upgrade pip
python -m pip install -r requirements.txt
python api_check.py
pause
