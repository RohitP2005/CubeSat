import { useEffect, useMemo, useRef, useState } from "react";
import { Canvas, useFrame, useThree } from "@react-three/fiber";
import * as THREE from "three";
import { CubeSatMesh } from "./CubeSatMesh";
import { useAttitudeStore } from "@/stores/attitudeStore";

type Source = "ekf" | "triad" | "ground_truth";

type Props = {
  source: Source;
  showGhost: boolean;
};

export function AttitudeScene({ source, showGhost }: Props) {
  const frame = useAttitudeStore((s) => s.latest);

  const primary =
    source === "ekf"
      ? frame?.ekf.quaternion
      : source === "triad"
        ? frame?.triad?.quaternion
        : frame?.ground_truth.quaternion;

  const ghost = frame?.ground_truth.quaternion;

  const [mounted, setMounted] = useState(false);
  useEffect(() => setMounted(true), []);
  if (!mounted) return <div className="h-full w-full" />;

  return (
    <Canvas
      shadows
      camera={{ position: [2.6, 2.1, 2.6], fov: 38 }}
      dpr={[1, 2]}
      gl={{ antialias: true }}
    >
      <color attach="background" args={["#0b1220"]} />
      <fog attach="fog" args={["#0b1220", 6, 14]} />
      <ambientLight intensity={0.35} />
      <directionalLight
        position={[4, 5, 3]}
        intensity={1.1}
        castShadow
        shadow-mapSize-width={1024}
        shadow-mapSize-height={1024}
      />
      <directionalLight position={[-3, 2, -2]} intensity={0.35} color="#7aa9ff" />

      <Stars count={1500} radius={50} />
      <GridPlane y={-1.2} size={20} divisions={20} />

      {showGhost && ghost && <CubeSatMesh quaternion={ghost} variant="ghost" />}
      {primary && (
        <CubeSatMesh
          quaternion={primary}
          variant="solid"
          color={source === "ekf" ? "#60a5fa" : source === "triad" ? "#fbbf24" : "#e2e8f0"}
        />
      )}

      <OrbitCamera minDistance={2} maxDistance={8} />
    </Canvas>
  );
}

/** Star field with circular soft dots and size variation. */
function Stars({ count = 1500, radius = 50 }: { count?: number; radius?: number }) {
  const [geometry, sizes, texture] = useMemo(() => {
    const positions = new Float32Array(count * 3);
    const sizeArr = new Float32Array(count);
    for (let i = 0; i < count; i++) {
      const u = Math.random();
      const v = Math.random();
      const theta = 2 * Math.PI * u;
      const phi = Math.acos(2 * v - 1);
      const r = radius * (0.8 + Math.random() * 0.2);
      positions[i * 3] = r * Math.sin(phi) * Math.cos(theta);
      positions[i * 3 + 1] = r * Math.sin(phi) * Math.sin(theta);
      positions[i * 3 + 2] = r * Math.cos(phi);
      sizeArr[i] = 0.25 + Math.random() * 0.55; // varied sizes
    }

    // Build a soft radial gradient texture (circle with falloff)
    const size = 64;
    const canvas = document.createElement("canvas");
    canvas.width = size;
    canvas.height = size;
    const ctx = canvas.getContext("2d")!;
    const grad = ctx.createRadialGradient(
      size / 2, size / 2, 0,
      size / 2, size / 2, size / 2
    );
    grad.addColorStop(0, "rgba(255,255,255,1)");
    grad.addColorStop(0.4, "rgba(255,255,255,0.8)");
    grad.addColorStop(1, "rgba(255,255,255,0)");
    ctx.fillStyle = grad;
    ctx.fillRect(0, 0, size, size);
    const tex = new THREE.CanvasTexture(canvas);
    tex.needsUpdate = true;

    const g = new THREE.BufferGeometry();
    g.setAttribute("position", new THREE.BufferAttribute(positions, 3));
    g.setAttribute("size", new THREE.BufferAttribute(sizeArr, 1));
    return [g, sizeArr, tex] as const;
  }, [count, radius]);

  const material = useMemo(() => {
    const mat = new THREE.PointsMaterial({
      color: "#e8efff",
      size: 1,
      map: texture,
      transparent: true,
      opacity: 0.95,
      sizeAttenuation: true,
      alphaTest: 0.02,
      depthWrite: false,
      fog: false,
    });
    return mat;
  }, [texture]);

  return (
    <points geometry={geometry}>
      <primitive object={material} attach="material" />
    </points>
  );
}

/** Grid plane using THREE.GridHelper. */
function GridPlane({ y = 0, size = 20, divisions = 20 }: { y?: number; size?: number; divisions?: number }) {
  const grid = useMemo(() => new THREE.GridHelper(size, divisions, "#2c3a5e", "#1f2a44"), [size, divisions]);
  return <primitive object={grid} position={[0, y, 0]} />;
}

/**
 * Minimal orbit camera: left-drag to rotate, wheel to zoom. Mimics drei's OrbitControls just enough.
 */
function OrbitCamera({ minDistance = 2, maxDistance = 8 }: { minDistance?: number; maxDistance?: number }) {
  const { camera, gl } = useThree();
  const state = useRef({
    azimuth: Math.atan2(2.6, 2.6),
    polar: Math.acos(2.1 / Math.hypot(2.6, 2.1, 2.6)),
    distance: Math.hypot(2.6, 2.1, 2.6),
    target: new THREE.Vector3(0, 0, 0),
    dragging: false,
    lastX: 0,
    lastY: 0,
    velAz: 0,
    velPo: 0,
  });

  useEffect(() => {
    const dom = gl.domElement;
    const onDown = (e: PointerEvent) => {
      state.current.dragging = true;
      state.current.lastX = e.clientX;
      state.current.lastY = e.clientY;
      dom.setPointerCapture(e.pointerId);
    };
    const onUp = (e: PointerEvent) => {
      state.current.dragging = false;
      try {
        dom.releasePointerCapture(e.pointerId);
      } catch {}
    };
    const onMove = (e: PointerEvent) => {
      if (!state.current.dragging) return;
      const dx = e.clientX - state.current.lastX;
      const dy = e.clientY - state.current.lastY;
      state.current.lastX = e.clientX;
      state.current.lastY = e.clientY;
      state.current.velAz = -dx * 0.005;
      state.current.velPo = -dy * 0.005;
    };
    const onWheel = (e: WheelEvent) => {
      e.preventDefault();
      const f = Math.pow(1.0015, e.deltaY);
      state.current.distance = THREE.MathUtils.clamp(
        state.current.distance * f,
        minDistance,
        maxDistance,
      );
    };
    dom.addEventListener("pointerdown", onDown);
    dom.addEventListener("pointerup", onUp);
    dom.addEventListener("pointermove", onMove);
    dom.addEventListener("wheel", onWheel, { passive: false });
    return () => {
      dom.removeEventListener("pointerdown", onDown);
      dom.removeEventListener("pointerup", onUp);
      dom.removeEventListener("pointermove", onMove);
      dom.removeEventListener("wheel", onWheel);
    };
  }, [gl, minDistance, maxDistance]);

  useFrame(() => {
    const s = state.current;
    s.azimuth += s.velAz;
    s.polar = THREE.MathUtils.clamp(s.polar + s.velPo, 0.15, Math.PI - 0.15);
    s.velAz *= 0.85;
    s.velPo *= 0.85;
    const r = s.distance;
    const x = r * Math.sin(s.polar) * Math.sin(s.azimuth);
    const y = r * Math.cos(s.polar);
    const z = r * Math.sin(s.polar) * Math.cos(s.azimuth);
    camera.position.set(x + s.target.x, y + s.target.y, z + s.target.z);
    camera.lookAt(s.target);
  });

  return null;
}
