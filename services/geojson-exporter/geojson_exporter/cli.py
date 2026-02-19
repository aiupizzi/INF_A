from __future__ import annotations

import argparse
import json
from pathlib import Path

from .exporter import export_geojson


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description='Export transformed defects as GeoJSON + artifact metadata.')
    parser.add_argument('--defects', required=True, help='Input JSON array of local defect detections.')
    parser.add_argument('--mapping-config', required=True, help='Coordinate mapping JSON config.')
    parser.add_argument('--output', required=True, help='Output GeoJSON file path.')
    parser.add_argument('--artifact-metadata', required=True, help='Output artifact metadata JSON path.')
    return parser


def main() -> None:
    args = build_parser().parse_args()
    metadata = export_geojson(
        defects_path=Path(args.defects),
        mapping_config_path=Path(args.mapping_config),
        output_path=Path(args.output),
        artifact_path=Path(args.artifact_metadata),
    )
    print(json.dumps(metadata, indent=2))


if __name__ == '__main__':
    main()
