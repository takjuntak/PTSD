// src/components/ThreeRobot.tsx
import { Canvas } from '@react-three/fiber';
import { OrbitControls, useGLTF } from '@react-three/drei';
import { Suspense, useRef } from 'react';
import { useFrame } from '@react-three/fiber';
import { Group } from 'three';

interface ThreeRobotProps {
  scale?: number;
  className?: string; // optional: 외부 div 스타일 조정용
}

const RobotModel = ({ scale = 1.0 }: { scale?: number }) => {
  const group = useRef<Group>(null);
  const { scene } = useGLTF('/robot.glb');
  scene.position.set(0, -1, 0);
  useFrame(() => {
    if (group.current) {
      group.current.rotation.y += 0.015;
    }
  });

  return <primitive ref={group} object={scene} scale={scale} />;
};

const ThreeRobot = ({ scale = 1.0, className = 'w-64 h-64' }: ThreeRobotProps) => (
  <div className={className}>
    <Canvas camera={{ position: [0, 1, 5], fov: 30 }} style={{ width: '100%', height: '100%' }}>
      <ambientLight intensity={0.6} />
      <directionalLight position={[5, 5, 5]} intensity={1} />
      <Suspense fallback={null}>
        <RobotModel scale={scale} />
      </Suspense>
      <OrbitControls enableZoom={false} enablePan={false} autoRotate={false} />
    </Canvas>
  </div>
);

export default ThreeRobot;
