import pybullet as p
import pybullet_data
import time

# Parámetros de la simulación
g = -9.8 
h_inicial = 3.0
radio = 0.5
dt = 1.0 / 240.0

physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())

planeId = p.loadURDF("plane.urdf")

# Crear una esfera en la posición inicial
startPosition = [0, 0, h_inicial]
startOrientation = [0, 0, 0, 1]
collisionShapeId = p.createCollisionShape(p.GEOM_SPHERE, radius=radio)
sphereId = p.createMultiBody(
    baseMass=1,
    baseCollisionShapeIndex=collisionShapeId,
    basePosition=startPosition,
    baseOrientation=startOrientation
)

# Desactivar la física interna de PyBullet
p.setGravity(0, 0, 0)

# Variables para el MRUA
h = h_inicial
vi = 0  # Velocidad inicial

# Bucle de simulación
while h > radio:  # Simula hasta que la esfera colisione con el suelo
    # Actualizar la velocidad y la posición usando las ecuaciones del MRUA
    vi += g * dt  # v = v0 + at
    h += vi * dt  # h = h0 + vt

    # Asegurarse de que no pase por debajo del suelo
    if h < radio:
        h = radio
        vi = 0

    # Actualizar la posición de la esfera en PyBullet
    p.resetBasePositionAndOrientation(sphereId, [0, 0, h], startOrientation)

    # Avanzar la simulación
    p.stepSimulation()
    time.sleep(dt)

# Desconectar PyBullet
p.disconnect()
