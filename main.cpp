#include <SFML/Window.hpp>
#include <SFML/OpenGL.hpp>
#include <cmath>
#include <optional>

// ------------------------------------------------------------------
// Mini quaternion propio (reemplaza a btQuaternion / btTransform)
// ------------------------------------------------------------------
struct Quat {
    float x = 0.f, y = 0.f, z = 0.f, w = 1.f;

    Quat normalized() const {
        float len = std::sqrt(x*x + y*y + z*z + w*w);
        if (len < 1e-8f) return Quat{0,0,0,1};
        return Quat{ x/len, y/len, z/len, w/len };
    }

    // Integra la rotación dado un vector de velocidad angular (rad/s) y dt.
    // Método: q' = q + 0.5 * (omega_quat * q) * dt, luego normalizar.
    void integrate(float wx, float wy, float wz, float dt) {
        Quat omega{ wx, wy, wz, 0.f };
        Quat dq = multiply(omega, *this);
        x += 0.5f * dq.x * dt;
        y += 0.5f * dq.y * dt;
        z += 0.5f * dq.z * dt;
        w += 0.5f * dq.w * dt;
        *this = normalized();
    }

    static Quat multiply(const Quat& a, const Quat& b) {
        return Quat{
            a.w*b.x + a.x*b.w + a.y*b.z - a.z*b.y,
            a.w*b.y - a.x*b.z + a.y*b.w + a.z*b.x,
            a.w*b.z + a.x*b.y - a.y*b.x + a.z*b.w,
            a.w*b.w - a.x*b.x - a.y*b.y - a.z*b.z
        };
    }

    // Genera una matriz 4x4 en formato column-major, igual que
    // btTransform::getOpenGLMatrix, lista para glMultMatrixf.
    void toOpenGLMatrix(float* m, float px = 0.f, float py = 0.f, float pz = 0.f) const {
        float xx = x*x, yy = y*y, zz = z*z;
        float xy = x*y, xz = x*z, yz = y*z;
        float wx = w*x, wy = w*y, wz = w*z;

        m[0] = 1.f - 2.f*(yy + zz);
        m[1] = 2.f*(xy + wz);
        m[2] = 2.f*(xz - wy);
        m[3] = 0.f;

        m[4] = 2.f*(xy - wz);
        m[5] = 1.f - 2.f*(xx + zz);
        m[6] = 2.f*(yz + wx);
        m[7] = 0.f;

        m[8]  = 2.f*(xz + wy);
        m[9]  = 2.f*(yz - wx);
        m[10] = 1.f - 2.f*(xx + yy);
        m[11] = 0.f;

        m[12] = px;
        m[13] = py;
        m[14] = pz;
        m[15] = 1.f;
    }
};

int main() {
    // ------------------------------------------------------------------
    // "Cuerpo rígido" simplificado: solo orientación + velocidad angular
    // ------------------------------------------------------------------
    Quat orientation; // identidad
    float angVelX = 0.5f, angVelY = 1.0f, angVelZ = 0.3f; // rad/s, igual que antes

    // ------------------------------------------------------------------
    // Ventana SFML 3
    // ------------------------------------------------------------------
    sf::Window window(sf::VideoMode({800, 600}), "SFML 3 (sin Bullet): Cubo Rotando");
    window.setFramerateLimit(60);

    // ------------------------------------------------------------------
    // OpenGL: cámara y proyección
    // ------------------------------------------------------------------
    glEnable(GL_DEPTH_TEST);

    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    float aspect = 800.f / 600.f;
    glFrustum(-aspect * 0.1f, aspect * 0.1f, -0.1f, 0.1f, 0.1f, 100.0f);

    const float dt = 1.f / 60.f;

    // ------------------------------------------------------------------
    // Bucle principal
    // ------------------------------------------------------------------
    while (window.isOpen()) {
        while (const std::optional event = window.pollEvent()) {
            if (event->is<sf::Event::Closed>())
                window.close();
        }

        // "Paso de física" manual: solo integramos la rotación
        orientation.integrate(angVelX, angVelY, angVelZ, dt);

        glClearColor(0.1f, 0.1f, 0.1f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        glMatrixMode(GL_MODELVIEW);
        glLoadIdentity();
        glTranslatef(0, 0, -10);

        float m[16];
        orientation.toOpenGLMatrix(m, 0.f, 0.f, 0.f);
        glMultMatrixf(m);

        // Cubo en wireframe verde (igual que el original)
        glBegin(GL_LINES);
            glColor3f(0.0f, 1.0f, 0.0f);
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

    return 0;
}