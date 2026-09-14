# Upstream data

Raw FlyWire FAFB v783 tables are not committed because they are upstream source data rather than generated project output. Zaku currently has:

- `classification.csv.gz` — 934,402 bytes
- `consolidated_cell_types.csv.gz` — 901,707 bytes
- `connections.csv.gz` — 50,289,304 bytes

The reproducible extractor consumes those files from `data/fafb783/`.
