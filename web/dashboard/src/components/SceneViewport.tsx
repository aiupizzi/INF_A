import { useEffect, useMemo, useRef, useState } from 'react';
import * as THREE from 'three';
import type { DefectFeature, DefectGeoJSON } from '../types/dashboard';

type ReconstructionFormat = 'mesh' | 'tiles';

interface SceneViewportProps {
  modelAssetUrl: string;
  tilesManifestUrl?: string;
  defects: DefectGeoJSON;
  selectedDefectId?: string;
  onSelectDefect: (defect: DefectFeature) => void;
}

const severityColor: Record<string, string> = {
  low: '#4ade80',
  medium: '#facc15',
  high: '#fb923c',
  critical: '#ef4444'
};

export const SceneViewport = ({
  modelAssetUrl,
  tilesManifestUrl,
  defects,
  selectedDefectId,
  onSelectDefect
}: SceneViewportProps) => {
  const mountRef = useRef<HTMLDivElement | null>(null);
  const [format, setFormat] = useState<ReconstructionFormat>('mesh');

  useEffect(() => {
    if (!mountRef.current) {
      return;
    }

    const mountEl = mountRef.current;
    const width = mountEl.clientWidth;
    const height = mountEl.clientHeight;

    const scene = new THREE.Scene();
    scene.background = new THREE.Color('#0b1020');

    const camera = new THREE.PerspectiveCamera(60, width / height, 0.1, 200);
    camera.position.set(12, 8, 14);
    camera.lookAt(6, 2, 0);

    const renderer = new THREE.WebGLRenderer({ antialias: true });
    renderer.setSize(width, height);
    mountEl.innerHTML = '';
    mountEl.appendChild(renderer.domElement);

    scene.add(new THREE.AmbientLight('#ffffff', 0.9));
    const directional = new THREE.DirectionalLight('#ffffff', 0.7);
    directional.position.set(4, 10, 5);
    scene.add(directional);

    // Placeholder model shell for mesh/tiles pipeline handoff.
    const baseGeometry = new THREE.BoxGeometry(12, 3, 5);
    const baseMaterial = new THREE.MeshStandardMaterial({ color: '#334155' });
    const reconstructedModel = new THREE.Mesh(baseGeometry, baseMaterial);
    reconstructedModel.position.set(6, 1.5, 0);
    scene.add(reconstructedModel);

    const pinGroup = new THREE.Group();
    defects.features.forEach((feature) => {
      const sphere = new THREE.Mesh(
        new THREE.SphereGeometry(0.2, 16, 16),
        new THREE.MeshStandardMaterial({ color: severityColor[feature.properties.severity] ?? '#22d3ee' })
      );
      const [x, y, z = 0.5] = feature.geometry.coordinates;
      sphere.position.set(x, y, z);
      pinGroup.add(sphere);
    });
    scene.add(pinGroup);

    const animate = () => {
      renderer.render(scene, camera);
      requestAnimationFrame(animate);
    };

    const resizeObserver = new ResizeObserver(() => {
      const nextWidth = mountEl.clientWidth;
      const nextHeight = mountEl.clientHeight;
      camera.aspect = nextWidth / nextHeight;
      camera.updateProjectionMatrix();
      renderer.setSize(nextWidth, nextHeight);
    });
    resizeObserver.observe(mountEl);

    animate();

    return () => {
      resizeObserver.disconnect();
      renderer.dispose();
      mountEl.innerHTML = '';
    };
  }, [defects, format]);

  const defectList = useMemo(() => defects.features, [defects]);

  return (
    <section className="card scene-card">
      <header className="scene-header">
        <div>
          <h2>3D Reconstruction Scene</h2>
          <p>
            Loading source: <code>{format === 'mesh' ? modelAssetUrl : tilesManifestUrl ?? 'No tileset configured'}</code>
          </p>
        </div>
        <div className="chip-group">
          <button className={format === 'mesh' ? 'chip active' : 'chip'} onClick={() => setFormat('mesh')}>
            Mesh
          </button>
          <button className={format === 'tiles' ? 'chip active' : 'chip'} onClick={() => setFormat('tiles')}>
            3D Tiles
          </button>
        </div>
      </header>

      <div className="scene-body">
        <div className="scene-canvas" ref={mountRef} />
        <aside className="pin-layer">
          <h3>GeoJSON Defect Pins</h3>
          <ul>
            {defectList.map((feature) => (
              <li key={feature.properties.id}>
                <button
                  className={selectedDefectId === feature.properties.id ? 'pin active' : 'pin'}
                  onClick={() => onSelectDefect(feature)}
                >
                  <span>{feature.properties.className}</span>
                  <strong>{feature.properties.severity.toUpperCase()}</strong>
                  <small>{Math.round(feature.properties.confidence * 100)}% confidence</small>
                </button>
              </li>
            ))}
          </ul>
        </aside>
      </div>
    </section>
  );
};
