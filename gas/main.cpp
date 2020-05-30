#include <cmath>
#include <iostream>
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
    virtual void process(const sf::Time& time) {}
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
            collectMomentum(p);
            p.y = 2 * y_ - p.y;
            p.vy *= -1;
        }
    }

    virtual void collectMomentum(Particle& p) {}

    void draw(sf::RenderWindow& window) override {
        sf::Vertex line[] =
        {
            sf::Vertex(sf::Vector2f(-1000, y_)),
            sf::Vertex(sf::Vector2f(1000, y_))
        };

        window.draw(line, 2, sf::Lines);
    };

protected:
    double y_;
    int dir_;
};

class MovingYWall : public YWall {
public:
static const constexpr double LAMBDA = 0;
//static const constexpr double ALPHA = 0.95;
static const constexpr double M = 1000;
static const constexpr double Mp = 200;
    MovingYWall(double y0, int dir, double v, bool accelerate) :
        YWall(y0, dir), v_(v), accelerate_(accelerate), totalTime_(sf::seconds(0)) {}
    void process(const sf::Time& time) {
        y_ += v_ * time.asSeconds();
        v_ *= (1 - LAMBDA * time.asSeconds());
        pressure_ = 0;
    }

    void interact(Particle& p) override {
        if ((p.y - y_) * dir_ < 0) {
            p.y = 2 * y_ - p.y;
            double vc = v_;
            double v0 = (M * v_ + Mp * p.vy) / (M + Mp);
            p.vy = 2 * v0 - p.vy;
            double vv = 2 * v0 - 2 * v_;
            pressure_ -= vv * M;
        }
    }

    double y() { return y_; }

    double pressure() { return pressure_; }

    void addV(double dv) {
        v_ += dv;
    }

private:
    double v_;
    bool accelerate_;
    double pressure_ = 0;
    sf::Time totalTime_;
};

class Piston {
public:
    static const constexpr double WIDTH = 10;
    static const constexpr double K = 10;
    Piston(double y, double m): m_(m), y0_(y), walls{std::make_shared<MovingYWall>(y - WIDTH / 2, -1, 0, true), std::make_shared<MovingYWall>(y + WIDTH / 2, 1, 0, true)} {};

    std::shared_ptr<MovingYWall> wall(int idx) { return walls[idx]; }

    void process(const sf::Time& time) {
        double y = (wall(0)->y() + wall(1)->y())/2;
        double totalP = walls[0]->pressure() + walls[1]->pressure();
        totalP += K * (y - y0_) * time.asSeconds() * MovingYWall::M;
        for (auto& wall : walls) {
            wall->addV(- totalP  / MovingYWall::M);
        }
    }

private:
    double m_, y0_;
    std::array<std::shared_ptr<MovingYWall>, 2> walls;
};


class PressureCollectorYWall : public YWall {
    static const constexpr double LAMBDA = 0.2;
public:
    using YWall::YWall;

    void process(const sf::Time& time) override {
        pressure_ *= (1 - LAMBDA * time.asSeconds());
        cnt_ *= (1 - LAMBDA * time.asSeconds());
        cnt_ += 1;
    }

    void collectMomentum(Particle& p) override {
        pressure_ += -p.vy * dir_;
    }

    double pressure() {
        return pressure_ / cnt_;
    }
private:
    double pressure_ = 0;
    double cnt_ = 0;
};

class Distribution {
public:
    Distribution(double x1, double y1, double x2, double y2, double vmax) :
        gen_(time(0)), vmax_(vmax),
        dx_(x1, x2), dy_(y1, y2), dv_(-vmax, vmax), dvi_(0, 1)
        {}

    Particle generate() {
        return Particle{
            dx_(gen_),
            dy_(gen_),
            0, // dv_(gen_),
            dv_(gen_)
            //vmax_ * (2 * dvi_(gen_) - 1),
            //vmax_ * (2 * dvi_(gen_) - 1)
        };
    }

private:
    std::mt19937 gen_;
    double vmax_;
    std::uniform_real_distribution<double> dx_, dy_, dv_;
    std::uniform_int_distribution<int> dvi_;
};

class Gas {
    static const constexpr int RADIUS = 3;
    static const constexpr double FORCE = 1;
    static const constexpr double D = 10;
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
            shape.setPosition(p.x - RADIUS, p.y - RADIUS);
            shape.setFillColor(sf::Color::Green);
            window.draw(shape);
        }
    }

    double energy() {
        double e = 0;
        for (const auto& p : particles_) {
            e += p.vx * p.vx + p.vy * p.vy;
        }
        return e / particles_.size();
    }

    void process(const sf::Time& time) {
        double sec = time.asSeconds();
        for (auto& p : particles_) {
            p.x += p.vx * sec;
            p.y += p.vy * sec;
            for (auto& b : boundaries_) {
                b->interact(p);
            }
        }
        /*
        for (int i = 0; i < particles_.size(); i++) {
            for (int j = 0; j < i; j++) {
                Particle& p1 = particles_[i];
                Particle& p2 = particles_[j];
                double d = std::hypot(p1.x - p2.x, p1.y - p2.y);
                if (d < D) {
                    continue;
                }
                double force = FORCE / d / d;
                double fx = force * (p1.x - p2.x) / d;
                double fy = force * (p1.y - p2.y) / d;
                p1.vx -= fx * sec;
                p1.vy -= fy * sec;
                p2.vx += fx * sec;
                p2.vy += fy * sec;
            }
        }
        */
    }

    const std::vector<Particle>& particles() const {
        return particles_;
    }

private:
    std::vector<std::shared_ptr<Boundary>> boundaries_;
    std::vector<Particle> particles_;
};

class Graph {
public:
    Graph(int sx, int sy, double maxx, double maxy, bool permanent) :
        sx_(sx), sy_(sy), maxx_(maxx), maxy_(maxy), permanent_(permanent), window_(sf::VideoMode(sx, sy), "Plot"), colors_{sf::Color::White, sf::Color::Red, sf::Color::Green, sf::Color::Cyan}, points_(colors_.size())
    {}

    void setPoint(int line, double x, double y) {
        points_[line].emplace_back(sf::Vector2f(x / maxx_ * sx_, sy_ - y / maxy_ * sy_), colors_[line]);
    }

    void draw() {
        window_.clear();
        for (auto& line: points_) {
            if (line.size()) {
                window_.draw(line.data(), line.size(), sf::LineStrip);
            }
            if (!permanent_) {
                line.clear();
            }
        }
        window_.display();
    }
private:
    int sx_, sy_;
    double maxx_, maxy_;
    bool permanent_;
    sf::RenderWindow  window_;
    std::vector<sf::Color> colors_;
    std::vector<std::vector<sf::Vertex>> points_;
};

class StatCollector {
    static const constexpr double LAMBDA = 0.3;
public:
    StatCollector(const Gas& gas, Graph& graph, double maxv, int bins):
        gas_(gas), graph_(graph), maxv_(maxv), bins_(bins), cnt_(bins_, 0), sumCnt_(0) {}

    void process(const sf::Time& time) {
        updateBins(time);
        calculateMaxwell();
        draw();
    }

    void draw() {
        /*double max = 0;
        for (const auto& v: {cnt_, maxwell_}) {
            for (const auto& x: v) {
                if (x > max) {
                    max = x;
                }
            }
        }*/
        double max = sumCnt_ / bins_ * 8;
        int idx = 0;
        for (const auto& v: {cnt_, maxwell_}) {
            for (int i = 0; i < bins_; i++) {
                graph_.setPoint(idx, i, v[i] / max);
            }
            idx++;
        }
    }

private:
    void updateBins(const sf::Time& time) {
        for (auto& x: cnt_) {
            x *= (1 - LAMBDA * time.asSeconds());
        }
        sumCnt_ *= (1 - LAMBDA * time.asSeconds());
        for (const auto& p: gas_.particles()) {
            double v = std::hypot(p.vx, p.vy);
            int bin = (v / maxv_) * bins_;
            if (bin > bins_) {
                continue;
            }
            cnt_[bin]++;
            sumCnt_++;
        }
    }

    void calculateMaxwell() {
        double sumVV = 0;
        double sumN = 0;
        for (int i = 0; i < cnt_.size(); i++) {
            sumVV += (i + 0.5) * (i + 0.5) * cnt_[i];
            sumN += cnt_[i];
        }
        double a = sumN / sumVV;
        double A = 2 * a * sumN;
        maxwell_.resize(cnt_.size());
        for (int i = 0; i < cnt_.size(); i++) {
            maxwell_[i] = A * i * std::exp(-1.0*i*i * a);
        }
    }

    const Gas& gas_;
    Graph& graph_;
    double maxv_;
    int bins_;
    std::vector<double> cnt_;
    std::vector<double> maxwell_;
    double sumCnt_;
};

int main()
{
    const double MARGIN = 10;
    const double MAXX = 800;
    const double MAXY = 800;
    const double MAXV = 100;
    const int N = 100;
    const int BINS = 100;
    const double MOVINGV = 10;
    sf::RenderWindow window(sf::VideoMode(MAXX + 10 * MARGIN, MAXY + MARGIN), "Gas");

    Piston piston(MAXY / 2, 1);
    auto pressureWall = std::make_shared<PressureCollectorYWall>(MAXY, -1);
    std::vector<std::shared_ptr<Boundary>> boundaries{
        std::make_shared<XWall>(MARGIN, 1),
        std::make_shared<XWall>(MAXX, -1),
        std::make_shared<YWall>(MARGIN, 1),
        piston.wall(0),
    };
    Distribution dist(0, MARGIN, MAXX, piston.wall(0)->y(), MAXV);
    Distribution dist2(0, piston.wall(1)->y(), MAXX, MAXY, MAXV / 2);

    Gas gas(N, dist, boundaries);

    std::vector<std::shared_ptr<Boundary>> boundaries2{
        std::make_shared<XWall>(MARGIN, 1),
        std::make_shared<XWall>(MAXX, -1),
        std::make_shared<YWall>(MAXY, -1),
        piston.wall(1)
    };
    Gas gas2(N, dist2, boundaries2);

    Graph graph(1300, MAXY, 3e3, MAXY, true);
    Graph graph2(1300, MAXY, 3e3, MAXV * MAXV, true);

    sf::Clock clock;
    sf::Time totalTime;

    while (window.isOpen())
    {
        sf::Event event;
        while (window.pollEvent(event))
        {
            if (event.type == sf::Event::Closed)
                window.close();
        }

        sf::Time elapsed = clock.restart();
        totalTime += elapsed;

        gas.process(elapsed);
        gas2.process(elapsed);

        piston.process(elapsed);

        for (auto& b: boundaries) {
            b->process(elapsed);
        }
        for (auto& b: boundaries2) {
            b->process(elapsed);
        }

        graph.setPoint(0, totalTime.asSeconds() * 30, MAXY - (piston.wall(0)->y() + piston.wall(1)->y()) / 2);
        graph.setPoint(1, totalTime.asSeconds() * 30, MAXY / 2);

        graph2.setPoint(0, totalTime.asSeconds() * 30, gas.energy());
        graph2.setPoint(1, totalTime.asSeconds() * 30, gas2.energy());

        window.clear();
        gas.draw(window);
        for (auto& b: boundaries) {
            b->draw(window);
        }
        gas2.draw(window);
        for (auto& b: boundaries2) {
            b->draw(window);
        }
        window.display();

        graph.draw();

        graph2.draw();
    }

    return 0;
}

