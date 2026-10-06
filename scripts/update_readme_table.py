"""Regenerate the README benchmark table from the results CSV.

Usage: python scripts/update_readme_table.py results/results.csv kalman-c [README.md]

Runs make_table.py and replaces everything between the
<!-- BENCH:START --> and <!-- BENCH:END --> markers.
"""
import os
import subprocess
import sys

START, END = "<!-- BENCH:START -->", "<!-- BENCH:END -->"

csv_path, ours = sys.argv[1], sys.argv[2]
readme = sys.argv[3] if len(sys.argv) > 3 else "README.md"
make_table = os.path.join(os.path.dirname(os.path.abspath(__file__)), "make_table.py")

table = subprocess.run([sys.executable, make_table, csv_path, ours],
                       check=True, capture_output=True, text=True).stdout

text = open(readme, encoding="utf-8").read()
if START not in text or END not in text:
    sys.exit(f"{readme} has no {START} ... {END} markers")
head, rest = text.split(START, 1)
_, tail = rest.split(END, 1)
open(readme, "w", encoding="utf-8").write(f"{head}{START}\n{table}{END}{tail}")
print(f"updated {readme}")
