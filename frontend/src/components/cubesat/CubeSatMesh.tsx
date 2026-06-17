import { useMemo, useRef } from "react";
import { useFrame } from "@react-three/fiber";
import * as THREE from "three";

type Props = {
  quaternion: [number, number, number, number] | null;
  variant?: "solid" | "ghost";
  color?: string;
};

/**
 * Stylized satellite: central bus + two solar panel wings + antenna dish + antenna boom.
 */
export function CubeSatMesh({ quaternion, variant = "solid", color = "#7aa9ff" }: Props) {
  const group = useRef<THREE.Group>(null);
  const target = useRef(new THREE.Quaternion());

  useFrame(() => {
    if (!group.current || !quaternion) return;
    // input is [w,x,y,z]; THREE expects (x,y,z,w)
    target.current.set(quaternion[1], quaternion[2], quaternion[3], quaternion[0]);
    group.current.quaternion.slerp(target.current, 0.35);
  });

  const opacity = variant === "ghost" ? 0.18 : 0.95;
  const ghost = variant === "ghost";

  const bodyMat = useMemo(
    () =>
      new THREE.MeshStandardMaterial({
        color: ghost ? "#94a3b8" : "#d6dce6",
        metalness: 0.6,
        roughness: 0.35,
        transparent: true,
        opacity,
      }),
    [ghost, opacity],
  );

  const accentMat = useMemo(
    () =>
      new THREE.MeshStandardMaterial({
        color: color,
        emissive: color,
        emissiveIntensity: ghost ? 0 : 0.25,
        metalness: 0.3,
        roughness: 0.4,
        transparent: true,
        opacity,
      }),
    [color, ghost, opacity],
  );

  const panelMat = useMemo(
    () =>
      new THREE.MeshStandardMaterial({
        color: "#1e3a8a",
        emissive: "#1e3a8a",
        emissiveIntensity: ghost ? 0 : 0.15,
        metalness: 0.7,
        roughness: 0.25,
        transparent: true,
        opacity,
      }),
    [ghost, opacity],
  );

  const frameMat = useMemo(
    () =>
      new THREE.MeshStandardMaterial({
        color: "#475569",
        metalness: 0.8,
        roughness: 0.3,
        transparent: true,
        opacity,
      }),
    [opacity],
  );

  // Solar panel with grid lines
  const panel = (xSign: number) => (
    <group position={[xSign * 1.05, 0, 0]}>
      {/* connecting arm */}
      <mesh material={frameMat} position={[-xSign * 0.35, 0, 0]}>
        <boxGeometry args={[0.3, 0.05, 0.05]} />
      </mesh>
      {/* panel */}
      <mesh material={panelMat}>
        <boxGeometry args={[1.2, 0.7, 0.04]} />
      </mesh>
      {/* grid frame */}
      {!ghost && (
        <lineSegments>
          <edgesGeometry args={[new THREE.BoxGeometry(1.2, 0.7, 0.04)]} />
          <lineBasicMaterial color="#7dd3fc" transparent opacity={0.55} />
        </lineSegments>
      )}
      {/* horizontal cell lines */}
      {!ghost &&
        [-0.2, 0, 0.2].map((y, i) => (
          <mesh key={`h${i}`} position={[0, y, 0.022]}>
            <boxGeometry args={[1.18, 0.005, 0.001]} />
            <meshBasicMaterial color="#0f172a" />
          </mesh>
        ))}
      {/* vertical cell lines */}
      {!ghost &&
        [-0.4, -0.2, 0, 0.2, 0.4].map((x, i) => (
          <mesh key={`v${i}`} position={[x, 0, 0.022]}>
            <boxGeometry args={[0.005, 0.68, 0.001]} />
            <meshBasicMaterial color="#0f172a" />
          </mesh>
        ))}
    </group>
  );

  return (
    <group ref={group}>
      {/* Main bus */}
      <mesh material={bodyMat} castShadow receiveShadow>
        <boxGeometry args={[0.7, 0.7, 0.9]} />
      </mesh>
      {!ghost && (
        <lineSegments>
          <edgesGeometry args={[new THREE.BoxGeometry(0.7, 0.7, 0.9)]} />
          <lineBasicMaterial color="#64748b" />
        </lineSegments>
      )}

      {/* Accent stripe (nose) */}
      <mesh material={accentMat} position={[0, 0, 0.46]}>
        <boxGeometry args={[0.72, 0.15, 0.02]} />
      </mesh>

      {/* Solar panels */}
      {panel(1)}
      {panel(-1)}

      {/* Antenna dish on +Z */}
      <group position={[0, 0.36, 0.2]}>
        <mesh material={frameMat}>
          <cylinderGeometry args={[0.02, 0.02, 0.2, 8]} />
        </mesh>
        <mesh position={[0, 0.16, 0]} rotation={[Math.PI, 0, 0]} material={bodyMat}>
          <coneGeometry args={[0.18, 0.1, 24, 1, true]} />
        </mesh>
      </group>

      {/* Antenna boom on -Z */}
      <mesh position={[0, 0, -0.6]} rotation={[Math.PI / 2, 0, 0]} material={frameMat}>
        <cylinderGeometry args={[0.015, 0.015, 0.3, 8]} />
      </mesh>
      <mesh position={[0, 0, -0.78]} material={accentMat}>
        <sphereGeometry args={[0.05, 16, 16]} />
      </mesh>

      {/* Body axes — only on solid version */}
      {variant === "solid" && (
        <>
          <BodyAxis dir={[1, 0, 0]} color="#ef4444" />
          <BodyAxis dir={[0, 1, 0]} color="#22c55e" />
          <BodyAxis dir={[0, 0, 1]} color="#3b82f6" />
        </>
      )}
    </group>
  );
}

function BodyAxis({ dir, color }: { dir: [number, number, number]; color: string }) {
  const len = 1.7;
  const end: [number, number, number] = [dir[0] * len, dir[1] * len, dir[2] * len];
  const mid: [number, number, number] = [end[0] / 2, end[1] / 2, end[2] / 2];

  // Orient cylinder along dir
  const quat = useMemo(() => {
    const up = new THREE.Vector3(0, 1, 0);
    const v = new THREE.Vector3(...dir).normalize();
    const q = new THREE.Quaternion().setFromUnitVectors(up, v);
    return q;
  }, [dir]);

  return (
    <group>
      <mesh position={mid} quaternion={quat}>
        <cylinderGeometry args={[0.015, 0.015, len, 12]} />
        <meshStandardMaterial color={color} emissive={color} emissiveIntensity={0.4} />
      </mesh>
      <mesh position={end} quaternion={quat}>
        <coneGeometry args={[0.05, 0.12, 16]} />
        <meshStandardMaterial color={color} emissive={color} emissiveIntensity={0.5} />
      </mesh>
    </group>
  );
}
