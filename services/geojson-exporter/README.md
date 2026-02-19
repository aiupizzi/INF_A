# geojson-exporter

Transforms local defect coordinates into the reconstruction coordinate reference and emits:

- GeoJSON `FeatureCollection` containing required defect properties.
- Artifact metadata JSON for dashboard ingestion.

## CLI

```bash
python -m geojson_exporter.cli \
  --defects ./defects.json \
  --mapping-config ./config/mapping.example.json \
  --output ./artifacts/defects.geojson \
  --artifact-metadata ./artifacts/defects.metadata.json
```

Metadata includes artifact path, feature count, CRS metadata, and mapping config path.
