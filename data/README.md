# Upstream data

Raw FlyWire FAFB v783 tables are intentionally not committed because they are upstream source data rather than project-generated outputs.

The extractor expects the following files under `data/fafb783/`:

- `classification.csv.gz`
- `consolidated_cell_types.csv.gz`
- `connections.csv.gz`

`extract_fafb_circuit.py` converts those upstream tables into the compact, checked-in circuit and report under `out/`.

The repository therefore keeps the reproducible extraction logic and derived research artifact while avoiding redistribution of the complete upstream dataset.