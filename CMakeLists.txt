#include <SFML/Window.hpp>
#include <SFML/OpenGL.hpp>
#include <btBulletDynamicsCommon.h>
#include <iostream>
#include <optional>

int main() {
    // 1. MUNDO FÍSICO
    auto* colConfig = new btDefaultCollisionConfiguration();
    auto* dispatcher = new btCollisionDispatcher(colConfig);
    auto* broadphase = new btDbvtBroadphase();
    auto* solver = new btSequentialImpulseConstraintSolver;
    auto* dynamicsWorld = new btDiscreteDynamicsWorld(dispatcher, broadphase, solver, colConfig);
    dynamicsWorld->setGravity(btVector3(0, -9.81f, 0));

    // 2. EL CUBO (Física)
    btCollisionShape* shape = new btBoxShape(btVector3(1, 1, 1));
    btTransform transform;
    transform.setIdentity();
    transform.setOrigin(btVector3(0, 5, 0)); // Altura inicial

    btVector3 inertia(0, 0, 0);
    shape->calculateLocalInertia(1.0f, inertia);

    auto* motionState = new btDefaultMotionState(transform);
    btRigidBody::btRigidBodyConstructionInfo rbInfo(1.0f, motionState, shape, inertia);
    auto* body = new btRigidBody(rbInfo);
    
    // Darle un pequeño impulso para que rote al caer
    body->setAngularVelocity(btVector3(1, 1, 0));
    dynamicsWorld->addRigidBody(body);

    // 3. VENTANA SFML 3
    sf::Window window(sf::VideoMode({800, 600}), "SFML 3 + Bullet");
    window.setFramerateLimit(60);

    // 4. BUCLE PRINCIPAL
    while (window.isOpen()) {
        // En SFML 3 pollEvent devuelve un std::optional
        while (const std::optional event = window.pollEvent()) {
            if (event->is<sf::Event::Closed>())
                window.close();
        }

        dynamicsWorld->stepSimulation(1.f / 60.f, 10);

        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        glMatrixMode(GL_MODELVIEW);
        glLoadIdentity();
        glTranslatef(0, 0, -15); // Alejar cámara

        btTransform trans;
        body->getMotionState()->getWorldTransform(trans);
        float m[16];
        trans.getOpenGLMatrix(m);
        glMultMatrixf(m);

        // Dibujar cubo simple
        glBegin(GL_LINES);
            glColor3f(0.0f, 1.0f, 0.0f);
            glVertex3f(-1,-1,-1); glVertex3f(1,-1,-1);
            glVertex3f(1,-1,-1);  glVertex3f(1,1,-1);
            glVertex3f(1,1,-1);   glVertex3f(-1,1,-1);
            glVertex3f(-1,1,-1);  glVertex3f(-1,-1,-1);
            glVertex3f(-1,-1,1);  glVertex3f(1,-1,1);
            glVertex3f(1,-1,1);   glVertex3f(1,1,1);
            glVertex3f(1,1,1);    glVertex3f(-1,1,1);
            glVertex3f(-1,1,1);   glVertex3f(-1,-1,1);
            glVertex3f(-1,-1,-1); glVertex3f(-1,-1,1);
            glVertex3f(1,-1,-1);  glVertex3f(1,-1,1);
            glVertex3f(1,1,-1);   glVertex3f(1,1,1);
            glVertex3f(-1,1,-1);  glVertex3f(-1,1,1);
        glEnd();

        window.display();
    }

    return 0;
}