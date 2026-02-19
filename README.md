# INF_A

Infrastructure for mission autonomy data flow.

## Added components

- `ros2_ws/src/infa_sync`: ROS 2 package that listens for mission completion, uploads collected imagery to WebODM via authenticated API, and polls reconstruction task status until completion or timeout.
- `services/geojson-exporter`: Python service/CLI that transforms local defect coordinates into reconstruction coordinates and exports GeoJSON + artifact metadata for dashboard ingestion.
- `docs/coordinate-transform-assumptions.md`: coordinate mapping assumptions and formula.

- `web/dashboard`: Vite + React dashboard for mission reconstruction, GeoJSON defect pinning, audit timeline, and Supabase audit-log integration.
