#include <SFML/Window.hpp>
#include <SFML/OpenGL.hpp>
#include <btBulletDynamicsCommon.h>
#include <iostream>
#include <optional>

int main() {
    // 1. CONFIGURACIÓN DE MUNDO FÍSICO (BULLET)
    auto* colConfig = new btDefaultCollisionConfiguration();
    auto* dispatcher = new btCollisionDispatcher(colConfig);
    auto* broadphase = new btDbvtBroadphase();
    auto* solver = new btSequentialImpulseConstraintSolver;
    auto* dynamicsWorld = new btDiscreteDynamicsWorld(dispatcher, broadphase, solver, colConfig);
    
    // Gravedad en 0 para que el cubo no se caiga y desaparezca de la cámara
    dynamicsWorld->setGravity(btVector3(0, 0, 0));

    // 2. CREACIÓN DEL CUBO FÍSICO
    btCollisionShape* shape = new btBoxShape(btVector3(1, 1, 1));
    // btCollisionShape* shape = new btBoxShape(btVector3(1,1,1));
    btTransform transform;
    transform.setIdentity();
    transform.setOrigin(btVector3(0, 0, 0)); // Centrado en el origen

    btVector3 inertia(0, 0, 0);
    shape->calculateLocalInertia(1.0f, inertia);

    auto* motionState = new btDefaultMotionState(transform);
    btRigidBody::btRigidBodyConstructionInfo rbInfo(1.0f, motionState, shape, inertia);
    auto* body = new btRigidBody(rbInfo);
    
    // Le damos una velocidad angular inicial para que rote solo
    body->setAngularVelocity(btVector3(0.5f, 1.0f, 0.3f));
    dynamicsWorld->addRigidBody(body);

    // 3. CONFIGURACIÓN DE VENTANA (SFML 3)
    sf::Window window(sf::VideoMode({800, 600}), "SFML 3 + Bullet: Cubo Rotando");
    window.setFramerateLimit(60);

    // 4. CONFIGURACIÓN DE OPENGL (Cámara y Luces)
    glEnable(GL_DEPTH_TEST);
    
    // Configurar la Proyección (Perspectiva)
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    float aspect = 800.f / 600.f;
    // glFrustum define el volumen de visión (izquierda, derecha, abajo, arriba, cerca, lejos)
    glFrustum(-aspect * 0.1f, aspect * 0.1f, -0.1f, 0.1f, 0.1f, 100.0f);

    // 5. BUCLE PRINCIPAL
    while (window.isOpen()) {
        // Manejo de eventos estilo SFML 3
        while (const std::optional event = window.pollEvent()) {
        // while (const std::optional event = window.pollEvent())
            if (event->is<sf::Event::Closed>())
            // if (event->is<sf::Event::Closed>())
                window.close();
        }

        // Paso de física
        dynamicsWorld->stepSimulation(1.f / 60.f, 10);

        // Limpiar pantalla con color gris oscuro
        glClearColor(0.1f, 0.1f, 0.1f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        glMatrixMode(GL_MODELVIEW);
        glLoadIdentity();

        // Alejamos la "cámara" 10 unidades para ver el cubo
        glTranslatef(0, 0, -10);

        // Obtener la matriz de transformación desde Bullet
        btTransform trans;
        body->getMotionState()->getWorldTransform(trans);
        float m[16];
        trans.getOpenGLMatrix(m);
        
        // Multiplicamos la matriz de OpenGL por la de Bullet
        glMultMatrixf(m);

        // DIBUJAR EL CUBO (Wireframe verde)
        glBegin(GL_LINES);
            glColor3f(0.0f, 1.0f, 0.0f); // Color verde
            // Aristas delanteras
            glVertex3f(-1,-1, 1); glVertex3f( 1,-1, 1);
            glVertex3f( 1,-1, 1); glVertex3f( 1, 1, 1);
            glVertex3f( 1, 1, 1); glVertex3f(-1, 1, 1);
            glVertex3f(-1, 1, 1); glVertex3f(-1,-1, 1);
            // Aristas traseras
            glVertex3f(-1,-1,-1); glVertex3f( 1,-1,-1);
            glVertex3f( 1,-1,-1); glVertex3f( 1, 1,-1);
            glVertex3f( 1, 1,-1); glVertex3f(-1, 1,-1);
            glVertex3f(-1, 1,-1); glVertex3f(-1,-1,-1);
            // Aristas laterales
            glVertex3f(-1,-1, 1); glVertex3f(-1,-1,-1);
            glVertex3f( 1,-1, 1); glVertex3f( 1,-1,-1);
            glVertex3f( 1, 1, 1); glVertex3f( 1, 1,-1);
            glVertex3f(-1, 1, 1); glVertex3f(-1, 1,-1);
        glEnd();

        window.display();
    }

    // 6. LIMPIEZA
    dynamicsWorld->removeRigidBody(body);
    delete body->getMotionState();
    delete body;
    delete shape;
    delete dynamicsWorld;
    delete solver;
    delete broadphase;
    delete dispatcher;
    delete colConfig;

    return 0;
}