# INF_A Dashboard

React + Vite app for viewing reconstruction outputs and defect audits.

## Features

- 3D scene viewport with a loading pipeline toggle for **mesh (`.glb`)** and **3D tiles (`tileset.json`)** sources.
- GeoJSON defect pin layer.
- Click-through interaction panel with hi-res image, confidence, severity, and suggested repair action.
- Filtering by severity, class, and minimum confidence.
- Mission selector and audit timeline.
- API adapter for Supabase-backed mission/audit/defect retrieval with mock fallback when env vars are missing.

## Environment

```bash
VITE_SUPABASE_URL=...
VITE_SUPABASE_ANON_KEY=...
```

When env vars are absent, mock data from `src/data/mockData.ts` is used.

## Loading & error-state strategy

`useDashboardData` centralizes all fetch orchestration and exposes:

- `state: 'idle' | 'loading' | 'success' | 'error'`
- `errorMessage`
- `refetch()`

`LoadStateBoundary` renders deterministic full-panel states:

1. **Loading:** mission + audit + defect payload status message.
2. **Error:** error details + retry button.
3. **Success:** dashboard content.

## Run

```bash
npm install
npm run dev
```
