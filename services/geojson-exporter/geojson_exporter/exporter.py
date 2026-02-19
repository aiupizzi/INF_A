from __future__ import annotations

import json
from dataclasses import dataclass
from pathlib import Path
from typing import Any



@dataclass
class Affine2DTransform:
    scale_x: float
    scale_y: float
    rotation_deg: float
    translation_x: float
    translation_y: float

    def apply(self, x_local: float, y_local: float) -> tuple[float, float]:
        from math import cos, radians, sin

        theta = radians(self.rotation_deg)
        x_scaled = x_local * self.scale_x
        y_scaled = y_local * self.scale_y
        x_rot = (x_scaled * cos(theta)) - (y_scaled * sin(theta))
        y_rot = (x_scaled * sin(theta)) + (y_scaled * cos(theta))
        return x_rot + self.translation_x, y_rot + self.translation_y


def load_mapping_config(path: Path) -> dict[str, Any]:
    return json.loads(path.read_text(encoding='utf-8'))


def transform_defects(defects: list[dict[str, Any]], mapping: dict[str, Any]) -> dict[str, Any]:
    affine = mapping['transform']['affine_2d']
    transform = Affine2DTransform(
        scale_x=float(affine['scale_x']),
        scale_y=float(affine['scale_y']),
        rotation_deg=float(affine['rotation_deg']),
        translation_x=float(affine['translation_x']),
        translation_y=float(affine['translation_y']),
    )

    features: list[dict[str, Any]] = []
    for defect in defects:
        x_world, y_world = transform.apply(
            float(defect['local_coordinates']['x']),
            float(defect['local_coordinates']['y']),
        )
        z_world = float(defect['local_coordinates'].get('z', 0.0))

        features.append(
            {
                'type': 'Feature',
                'geometry': {
                    'type': 'Point',
                    'coordinates': [x_world, y_world, z_world],
                },
                'properties': {
                    'defect_id': defect['defect_id'],
                    'severity': defect['severity'],
                    'confidence': defect['confidence'],
                    'image_path': defect.get('image_path') or defect.get('image_url'),
                    'suggested_action': defect['suggested_action'],
                },
            }
        )

    return {
        'type': 'FeatureCollection',
        'name': mapping.get('target_crs', {}).get('name', 'reconstruction_frame'),
        'features': features,
    }


def export_geojson(
    defects_path: Path,
    mapping_config_path: Path,
    output_path: Path,
    artifact_path: Path,
) -> dict[str, Any]:
    defects = json.loads(defects_path.read_text(encoding='utf-8'))
    mapping = load_mapping_config(mapping_config_path)
    collection = transform_defects(defects, mapping)

    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text(json.dumps(collection, indent=2), encoding='utf-8')

    metadata = {
        'artifact_type': 'geojson-defect-map',
        'artifact_path': str(output_path),
        'feature_count': len(collection['features']),
        'crs': mapping.get('target_crs', {}),
        'mapping_config': str(mapping_config_path),
    }
    artifact_path.parent.mkdir(parents=True, exist_ok=True)
    artifact_path.write_text(json.dumps(metadata, indent=2), encoding='utf-8')
    return metadata
