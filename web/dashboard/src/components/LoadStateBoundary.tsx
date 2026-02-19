import type { ReactNode } from 'react';
import type { LoadingState } from '../types/dashboard';

interface LoadStateBoundaryProps {
  state: LoadingState;
  errorMessage: string | null;
  onRetry: () => void;
  children: ReactNode;
}

export const LoadStateBoundary = ({
  state,
  errorMessage,
  onRetry,
  children
}: LoadStateBoundaryProps) => {
  if (state === 'loading' || state === 'idle') {
    return (
      <section className="card state-card">
        <h2>Loading dashboard data…</h2>
        <p>Fetching reconstruction metadata, defect GeoJSON, and audit logs.</p>
      </section>
    );
  }

  if (state === 'error') {
    return (
      <section className="card state-card error">
        <h2>Unable to load dashboard</h2>
        <p>{errorMessage ?? 'Unexpected data loading error.'}</p>
        <button onClick={onRetry}>Retry</button>
      </section>
    );
  }

  return <>{children}</>;
};
