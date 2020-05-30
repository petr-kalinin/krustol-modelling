#include <memory>
#include <random>
#include <vector>

#include <SFML/Graphics.hpp>

struct Particle {
    double x, y;
    double vx, vy;
};

class Boundary {
public:
    virtual void interact(Particle& particle) = 0;
    virtual void process() {}
    virtual void draw(sf::RenderWindow& window) = 0;
};

class XWall : public Boundary {
public:
    XWall(double x, int dir) :
        x_(x), dir_(dir) {}

    void interact(Particle& p) override {
        if ((p.x - x_) * dir_ < 0) {
            p.x = 2 * x_ - p.x;
            p.vx *= -1;
        }
    }

    void draw(sf::RenderWindow& window) override {
        sf::Vertex line[] =
        {
            sf::Vertex(sf::Vector2f(x_, -1000)),
            sf::Vertex(sf::Vector2f(x_, 1000))
        };

        window.draw(line, 2, sf::Lines);
    };

private:
    double x_;
    int dir_;
};

class YWall : public Boundary {
public:
    YWall(double y, int dir) :
        y_(y), dir_(dir) {}

    void interact(Particle& p) override {
        if ((p.y - y_) * dir_ < 0) {
            p.y = 2 * y_ - p.y;
            p.vy *= -1;
        }
    }

    void draw(sf::RenderWindow& window) override {
        sf::Vertex line[] =
        {
            sf::Vertex(sf::Vector2f(-1000, y_)),
            sf::Vertex(sf::Vector2f(1000, y_))
        };

        window.draw(line, 2, sf::Lines);
    };

private:
    double y_;
    int dir_;
};

class Distribution {
public:
    Distribution(double x1, double y1, double x2, double y2, double vmax) :
        gen_(time(0)),
        dx_(x1, x2), dy_(y1, y2), dv_(-vmax, vmax)
        {}

    Particle generate() {
        return Particle{
            dx_(gen_),
            dy_(gen_),
            dv_(gen_),
            dv_(gen_)
        };
    }

private:
    std::mt19937 gen_;
    std::uniform_real_distribution<double> dx_, dy_, dv_;
};

class Gas {
    static const constexpr int RADIUS = 3;
public:
    Gas(int n, Distribution& dist, std::vector<std::shared_ptr<Boundary>> boundaries) :
        boundaries_(std::move(boundaries))
    {
        particles_.reserve(n);
        for (int i = 0; i < n; i++) {
            particles_.push_back(dist.generate());
        }
    }

    void draw(sf::RenderWindow& window) {
        for (const auto& p : particles_) {
            sf::CircleShape shape(RADIUS);
            shape.setPosition(p.x, p.y);
            shape.setFillColor(sf::Color::Green);
            window.draw(shape);
        }
    }

    void process() {
        for (auto& p : particles_) {
            p.x += p.vx;
            p.y += p.vy;
            for (auto& b : boundaries_) {
                b->interact(p);
            }
        }
    }

private:
    std::vector<std::shared_ptr<Boundary>> boundaries_;
    std::vector<Particle> particles_;
};

int main()
{
    const double MARGIN = 10;
    const double MAXX = 800;
    const double MAXY = 800;
    const double MAXV = 0.2;
    sf::RenderWindow window(sf::VideoMode(MAXX + MARGIN, MAXY + MARGIN), "Gas");

    Distribution dist(0, 0, MAXX, MAXY, MAXV);
    std::vector<std::shared_ptr<Boundary>> boundaries{
        std::make_shared<XWall>(MARGIN, 1),
        std::make_shared<XWall>(MAXX, -1),
        std::make_shared<YWall>(MARGIN, 1),
        std::make_shared<YWall>(MAXY, -1),
    };
    Gas gas(20, dist, boundaries);

    while (window.isOpen())
    {
        sf::Event event;
        while (window.pollEvent(event))
        {
            if (event.type == sf::Event::Closed)
                window.close();
        }

        gas.process();
        for (auto& b: boundaries) {
            b->process();
        }
        window.clear();
        gas.draw(window);
        for (auto& b: boundaries) {
            b->draw(window);
        }
        window.display();
    }

    return 0;
}

