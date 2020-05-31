#include <cmath>
#include <iostream>
#include <memory>
#include <random>
#include <vector>

#include <SFML/Graphics.hpp>

#include <proj_api.h>

#include <osmium/handler.hpp>
#include <osmium/osm/types.hpp>
#include <osmium/index/map/dummy.hpp>
#include <osmium/index/map/sparse_mem_array.hpp>
#include <osmium/handler/node_locations_for_ways.hpp>
#include <osmium/io/file.hpp>
#include <osmium/area/assembler.hpp>
#include <osmium/area/multipolygon_collector.hpp>
#include <osmium/io/pbf_input.hpp>

bool g_running = true;
int g_rootId = -1;

struct point {
    double x,y;
};

class Projector {
public:
    Projector():
        latlonProj(pj_init_plus("+proj=latlong +datum=WGS84")),
        resultProj(pj_init_plus("+init=epsg:3857")) {}

    point transform(point p) const {
        p.x *= M_PI/180;
        p.y *= M_PI/180;
        pj_transform(latlonProj, resultProj, 1, 1, &p.x, &p.y, NULL);
        return p;
    }

    point invertTransform(point p) const {
        pj_transform(resultProj, latlonProj, 1, 1, &p.x, &p.y, NULL);
        p.x /= M_PI/180;
        p.y /= M_PI/180;
        return p;
    }

private:
    projPJ latlonProj;
    projPJ resultProj;
};

struct MinMax {
    double maxx, maxy, minx, miny;
};

class Window {
public:
    Window(int sx, int sy, const char* title) : window_(std::make_unique<sf::RenderWindow>(sf::VideoMode(sx, sy), title)) {}
    Window(Window&&) = default;

    sf::RenderWindow& window() { return *window_; }

    void process() {
        sf::Event event;
        while (window_->pollEvent(event))
        {
            if (event.type == sf::Event::Closed)
                g_running = false;
        }
    }
private:
    std::unique_ptr<sf::RenderWindow> window_;
};

class OsmHandler;
class BaseCar {
public:
    virtual void process(const sf::Time& elapsed) = 0;
    virtual void draw(sf::RenderWindow& window) = 0;
    virtual bool finished() const = 0;
    virtual double v() const = 0;
    virtual bool jams() const = 0;
    virtual bool update() const = 0;
};

class Graph {
public:
    struct RoadOptions {
        RoadOptions(double vmax = -1, int lanes = -1) : vmax(vmax), lanes(lanes) {}
        RoadOptions(const RoadOptions&) = default;
        RoadOptions& operator=(const RoadOptions&) = default;
        double length;
        double vmax;
        int lanes;
    };
    struct State {
        static const constexpr double LAMBDA = 0.2;
        State() : cars{}, sumV(0), sumCnt(0) {}
        std::vector<BaseCar*> cars;
        double sumV;
        double sumCnt;
        double v() { return (sumV + 60*0.01) / (sumCnt + 0.01); }
        void addV(double v) {
            sumV = sumV * (1 - LAMBDA) + v;
            sumCnt = sumCnt * (1 - LAMBDA) + 1;
        }
    };
    struct Edge {
        int from, to;
        std::vector<sf::Vertex> line;
        RoadOptions options;
        mutable State state;
    };
    Graph(int sx, int sy): sx_(sx), sy_(sy), window_(sx, sy, "Graph") {}
    void draw() {
        window_.window().clear();
        for (auto& pair: edges_) {
            for (auto& pair2: pair.second) {
                auto& edge = pair2.second;
                double v = edge.state.v();
                sf::Color color(255, v / 60 * 255, v / 60 * 255);
                for (auto& p : edge.line) {
                    p.color = color;
                }
                window_.window().draw(edge.line.data(), edge.line.size(), sf::LineStrip);
            }
        }
        for (const auto& car: cars_) {
            car->draw(window_.window());
        }
        window_.window().display();
    }
    void process(const sf::Time& elapsed) {
        window_.process();
        for (auto& car: cars_) {
            car->process(elapsed);
        }
        cars_.erase(std::remove_if(cars_.begin(), cars_.end(), [](const std::unique_ptr<BaseCar>& c) { return c->finished(); }), cars_.end());
        int nocc = 0;
        for (const auto& pair: edges_) {
            for (const auto& pair2: pair.second) {
                const auto& edge = pair2.second;
                nocc += edge.state.cars.size();
            }
        }
    }
    const std::map<int, sf::Vertex>& vertices() const { return vertices_; }
    const std::map<int, std::map<int, Edge>>& edges() const { return edges_; }
    void addEdge(int u, int v, std::vector<sf::Vertex> line, RoadOptions options) {
        options.length = 0;
        for (int i = 1; i < line.size(); i++) {
            options.length += std::hypot(line[i].position.x - line[i-1].position.x, line[i].position.y - line[i-1].position.y);
        }
        vertices_[u] = line.front();
        vertices_[v] = line.back();
        edges_[u][v] = {u, v, line, options};
        std::reverse(line.begin(), line.end());
        edges_[v][u] = {v, u, line, options};
    }
    void addEdge(const Edge& edge) {
        addEdge(edge.from, edge.to, edge.line, edge.options);
    }
    void addCar(std::unique_ptr<BaseCar> car) {
        cars_.emplace_back(std::move(car));
    }
    int carsCount() {
        return cars_.size();
    }
    const std::vector<std::unique_ptr<BaseCar>>& cars() { return cars_; }
    int sx() const { return sx_; }
    int sy() const { return sy_; }
    Window& window() { return window_; }

private:
    int sx_, sy_;
    std::map<int, std::map<int, Edge>> edges_;
    std::map<int, sf::Vertex> vertices_;
    Window window_;
    std::vector<std::unique_ptr<BaseCar>> cars_;
};

class GraphCompacter {
public:
    GraphCompacter(const Graph& source): source_(source), result_(source_.sx(), source_.sy()) {}
    Graph run(int startId) {
        dfs(startId, -1, {-1, -1, {}, {-1, -1}});
        return std::move(result_);
    }
private:
    void dfs(int vertex, int from, Graph::Edge edge) {
        if (!source_.vertices().count(vertex)) {
            std::cout << "Vertex " << vertex << " not found in source";
        }
        if (visited_.count(vertex)) {
            edge.to = vertex;
            result_.addEdge(edge);
            return;
        }
        visited_.insert(vertex);
        if (source_.edges().at(vertex).size() == 2) {
            for (const auto& pair: source_.edges().at(vertex)) {
                if (pair.first == from)
                    continue;
                if (edge.line.size())
                    edge.line.pop_back();
                edge.line.insert(edge.line.end(), pair.second.line.begin(), pair.second.line.end());
                dfs(pair.first, vertex, edge);
                return;
            }
        }
        if (!edge.line.empty()) {
            edge.to = vertex;
            result_.addEdge(edge);
        }
        for (const auto& pair: source_.edges().at(vertex)) {
            if (pair.first == from)
                continue;
            dfs(pair.first, vertex, pair.second);
        }
    }

    std::set<int> visited_;
    const Graph& source_;
    Graph result_;
};

class Algorithm {
    struct Info {
        int d;
        int from;
    };
    static const constexpr double FACTOR = 1e3;
public:
    Algorithm(const Graph& graph, bool jams): graph_(graph), jams_(jams) {}
    std::vector<int> getPath(int from, int to) {
        //return {from, to};
        graph_.vertices().at(from);
        graph_.vertices().at(to);
        std::set<std::pair<int, int>> q;
        std::map<int, Info> infos;
        q.emplace(0, from);
        infos[from] = {0, -1};
        while (!q.empty()) {
            auto p = *q.begin();
            q.erase(q.begin());
            int curv = p.second;
            for (const auto& pair: graph_.edges().at(curv)) {
                int newV = pair.first;
                double v = pair.second.options.vmax;
                if (jams_) {
                    v = std::min(v, pair.second.state.v());
                }
                int newD = infos[curv].d + pair.second.options.length / v * FACTOR;
                if (!infos.count(newV)) {
                    infos[newV] = {newD, curv};
                    q.emplace(newD, newV);
                    continue;
                }
                double oldD = infos[newV].d;
                if (oldD <= newD)
                    continue;
                q.erase({oldD, newV});
                q.emplace(newD, newV);
                infos[newV].d = newD;
                infos[newV].from = curv;
            }
        }
        std::vector<int> result;
        int curV = to;
        while (curV != from) {
            result.push_back(curV);
            curV = infos.at(curV).from;
        }
        result.push_back(curV);
        std::reverse(result.begin(), result.end());
        return result;
    };
protected:
    const Graph& graph_;
    bool jams_;
};

class Car : public BaseCar {
static const constexpr double VFACTOR = 0.3;
static const constexpr int RADIUS = 5;
static const constexpr double LENGTH = 15;
public:
    Car(const Graph& graph, int from, int to, bool jams, bool update): graph_(graph), to_(to), jams_(jams), update_(jams && update) {
        Algorithm algo(graph_, jams_);
        path_ = algo.getPath(from, to);
        index_ = 0;
        edgePart_ = -1;
    }

    void process(const sf::Time& elapsed) {
        if (index_ >= path_.size() - 1) {
            return;
        }
        int from = path_[index_];
        int to = path_[index_ + 1];
        const auto& edge = graph_.edges().at(from).at(to);
        if (edgePart_ < 0) {
            edgePart_ = 0;
            edge.state.cars.push_back(this);
            if (update_ && jams_) {
                Algorithm algo(graph_, jams_);
                path_ = algo.getPath(to, to_);
                path_.insert(path_.begin(), from);
                index_ = 0;
                assert(path_[index_] == from);
                assert(path_[index_ + 1] == to);
            }
        }
        int position = std::find(edge.state.cars.begin(), edge.state.cars.end(), this) - edge.state.cars.begin();
        double jam = (position - 1) * LENGTH / edge.options.lanes;

        if (edgePart_ + jam > edge.options.length && edgePart_ + LENGTH < edge.options.length) {
            jammed_ = true;
            timeSinceJammed_ = sf::seconds(0);
            v_ = 0;
            edge.state.addV(0);
            return;
        }
        jammed_ = false;
        double v = VFACTOR * edge.options.vmax;
        edgePart_ += v * elapsed.asSeconds();
        if (edgePart_ > edge.options.length) {
            if (index_ != path_.size() - 2) {
                int toto = path_[index_ + 2];
                const auto& edge2 = graph_.edges().at(to).at(toto);
                double jam2 = edge2.state.cars.size() * LENGTH / edge2.options.lanes;
                if (edge2.state.cars.size() && jam2 > edge2.options.length) {
                    edgePart_ -= v * elapsed.asSeconds();
                    timeSinceJammed_ = sf::seconds(0);
                    jammed_ = true;
                    v_ = 0;
                    edge.state.addV(0);
                    return;
                }
            }
            auto it = std::find(edge.state.cars.begin(), edge.state.cars.end(), this);
            edge.state.cars.erase(it);
            index_++;
            edgePart_ = -1;
        }
        timeSinceJammed_ += elapsed;
        v_ = edge.options.vmax;
        edge.state.addV(v_);
    }

    void draw(sf::RenderWindow& window) {
        /*for (const auto& id: {path_.front(), path_.back()}) {
            sf::CircleShape shape(RADIUS);
            auto start = graph_.vertices().at(id).position;
            shape.setPosition(start.x - RADIUS, start.y - RADIUS);
            shape.setFillColor(sf::Color::Cyan);
            window.draw(shape);
        }*/
        if (index_ >= path_.size() - 1) {
            return;
        }
        {
        sf::Vertex position = getPosition();
        //double r = RADIUS * (3 - timeSinceJammed_.asSeconds()) / 3;
        //if (r < 1) r = 1;
        double r = 3;
        sf::CircleShape shape(r);
        shape.setPosition(position.position.x - r, position.position.y - r);
        if (jams_) {
            if (jammed_)
                shape.setFillColor(sf::Color::Yellow);
            else
                shape.setFillColor(sf::Color::Cyan);
        } else {
            if (jammed_)
                shape.setFillColor(sf::Color::Red);
            else
                shape.setFillColor(sf::Color::Green);
        }
        window.draw(shape);
        }
    }

    bool finished() const override {
        return index_ >= path_.size() - 1;
    }

    sf::Vertex getPosition() {
        int from = path_[index_];
        int to = path_[index_ + 1];
        const auto& edge = graph_.edges().at(from).at(to);

        double passed = edgePart_;
        for (int i = 0; i < edge.line.size() - 1; i++) {
            double len = std::hypot(edge.line[i+1].position.x - edge.line[i].position.x,
                                    edge.line[i+1].position.y - edge.line[i].position.y);
            passed -= len;
            if (passed < 0) {
                double frac = -passed / len;
                double x = edge.line[i+1].position.x * (1 - frac) + edge.line[i].position.x * frac;
                double y = edge.line[i+1].position.y * (1 - frac) + edge.line[i].position.y * frac;
                return {sf::Vector2f(x, y)};
            }
        }
        return edge.line.back();
    }

    double v() const override { return v_; }
    bool jams() const override { return jams_; }
    bool update() const override { return update_; }

private:
    const Graph& graph_;
    int to_;
    std::vector<int> path_;
    int index_;
    double edgePart_;
    bool jammed_ = false;
    sf::Time timeSinceJammed_ = sf::seconds(100);
    double v_ = 0;
    bool jams_;
    bool update_;
};

class CarGenerator {
    static const constexpr double JAMS_FRACTION = 0.2;
    static const constexpr double UPDATE_FRACTION = 0.5;
public:
    CarGenerator(const Graph& graph) : graph_(graph), gen_(time(0)), dist_(0, graph.vertices().size() - 1), jams_(0, 1) {
        ids_.reserve(graph.vertices().size());
        for (const auto& v: graph.vertices()) {
            ids_.push_back(v.first);
        }
    }
    std::unique_ptr<Car> generate() {
        return std::make_unique<Car>(graph_, randomVertex(), randomVertex(), jams_(gen_) < JAMS_FRACTION, jams_(gen_) < UPDATE_FRACTION);
    }
    int randomVertex() {
        return ids_[dist_(gen_)];
    }
private:
    const Graph& graph_;
    std::vector<int> ids_;
    std::mt19937 gen_;
    std::uniform_int_distribution<int> dist_;
    std::uniform_real_distribution<double> jams_;
};

class OsmHandler {
    std::map<std::string, Graph::RoadOptions> OPTIONS{
        {"motorway", {110, 2}},
        {"motorway_link", {90, 2}},
        {"trunk", {90, 1}},
        {"trunk_link", {90, 1}},
        {"primary", {60, 1}},
        {"primary_link", {60, 1}},
        {"secondary", {60, 1}},
        {"secondary_link", {60, 1}},
        {"tertiary", {45, 1}},
        {"tertiary_link", {45, 1}},
        {"unclassified", {30, 1}},
        //{"residential", {30, 1}},
        //{"service", {BASE_WIDTH, RoadType::SIDE}},
        {"foo", {0, 0}}
    };
public:
    OsmHandler(const Projector& proj, const MinMax& minmax, int sx, int sy) : graph_(sx, sy), proj_(proj), minmax_(minmax), sx_(sx), sy_(sy), scale_(sx/(minmax_.maxx - minmax_.minx)) {}
    virtual void osm_object (const osmium::OSMObject &) {}
    virtual void node (const osmium::Node &) {}
    virtual void relation (const osmium::Relation &) {}
    virtual void area (const osmium::Area &) {}
    virtual void changeset (const osmium::Changeset &) {}
    virtual void tag_list (const osmium::TagList &) {}
    virtual void way_node_list (const osmium::WayNodeList &) {}
    virtual void relation_member_list (const osmium::RelationMemberList &) {}
    virtual void outer_ring (const osmium::OuterRing &) {}
    virtual void inner_ring (const osmium::InnerRing &) {}
    virtual void changeset_discussion (const osmium::ChangesetDiscussion &) {}
    virtual void flush() const {}
    virtual void way(const osmium::Way & way) {
        if (!way.get_value_by_key("highway"))
            return;
        std::string type = way.get_value_by_key("highway");
        auto option = OPTIONS.find(type);
        if (option == OPTIONS.end())
            return;
        bool first =  true;
        sf::Vertex prev;
        int prevId;
        for (const auto& node: way.nodes()) {
            if (!node.location())
                continue;
            point p = proj_.transform({node.lon(), node.lat()});
            double x = scale_ * (p.x-minmax_.minx);
            double y = scale_ * (minmax_.maxy-p.y);
            sf::Vertex vertex(sf::Vector2f(x, y));
            if (!first && x > 0 && x < sx_ && y > 0 && y < sy_
                && prev.position.x > 0 && prev.position.x < sx_ && prev.position.y > 0 && prev.position.y < sy_)
            {
                if (type == "primary") {
                    g_rootId = node.ref();
                }
                graph_.addEdge(prevId, node.ref(), {prev, vertex}, option->second);
            }
            first = false;
            prev = vertex;
            prevId = node.ref();
        }
    }
    Graph graph() { return std::move(graph_); }
private:
    Graph graph_;
    Projector proj_;
    MinMax minmax_;
    int sx_, sy_;
    double scale_;
};

class Plot {
public:
    Plot(int sx, int sy, double maxx, double maxy, bool permanent) :
        sx_(sx), sy_(sy), maxx_(maxx), maxy_(maxy), permanent_(permanent), window_(sf::VideoMode(sx, sy), "Plot"), colors_{sf::Color::White, sf::Color::Red, sf::Color::Green, sf::Color::Cyan}, points_(colors_.size())
    {}

    void setPoint(int line, double x, double y) {
        points_[line].emplace_back(sf::Vector2f(x / maxx_ * sx_, sy_ - y / maxy_ * sy_), colors_[line]);
        auto p = points_[line].back().position;
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

Graph load() {
    int SIZE = 2e4;
    int SIZEX = 800;
    int SIZEY = 800;
    const char* filename = "data/RU-NIZ.osm.pbf";

    typedef osmium::index::map::Dummy<osmium::unsigned_object_id_type, osmium::Location> IndexNeg;
    typedef osmium::index::map::SparseMemArray<osmium::unsigned_object_id_type, osmium::Location> IndexPos;
    typedef osmium::handler::NodeLocationsForWays<IndexPos, IndexNeg> LocationHandler;

    Projector proj;
    MinMax minmax;
    point center = proj.transform({43.943939, 56.279961});
    minmax.minx = center.x - SIZE;
    minmax.maxx = center.x + SIZE;
    minmax.miny = center.y - SIZE;
    minmax.maxy = minmax.miny + (minmax.maxx - minmax.minx);

    OsmHandler handler(proj, minmax, SIZEX, SIZEY);
    osmium::io::File infile(filename);

    osmium::area::Assembler::config_type assembler_config;
    osmium::area::MultipolygonCollector<osmium::area::Assembler> collector(assembler_config);

    std::cerr << "Pass 1...\n";
    osmium::io::Reader reader1(infile);
    collector.read_relations(reader1);
    reader1.close();
    std::cerr << "Pass 1 done\n";

    IndexPos indexPos;
    IndexNeg indexNeg;
    LocationHandler locationHandler(indexPos, indexNeg);
    locationHandler.ignore_errors();

    std::cerr << "Pass 2...\n";
    osmium::io::Reader reader2(infile);
    osmium::apply(reader2, locationHandler, handler, collector.handler([&handler](osmium::memory::Buffer&& buffer) {
        osmium::apply(buffer, handler);
    }));
    reader2.close();
    std::cerr << "Pass 2 done\n";

    return handler.graph();
}

int main()
{
    int cars = 0;

    Graph graph0 = load();
    GraphCompacter compacter(graph0);
    Graph graph = compacter.run(g_rootId);

    std::cout << "Initial graph =" << graph0.vertices().size() << std::endl;
    std::cout << "Reduced graph =" << graph.vertices().size() << std::endl;

    CarGenerator generator(graph);

    std::cout << "Start!";

    Plot plot(800, 800, 4000, 60, true);

    sf::Clock clock;
    sf::Time totalTime = sf::seconds(0);
    while (g_running)
    {
        sf::Time elapsed = clock.restart();
        totalTime += elapsed;

        cars = totalTime.asSeconds() * 10;

        while (graph.carsCount() < cars) {
            graph.addCar(generator.generate());
        }

        graph.process(elapsed);
        graph.draw();

        double sumV = 0;
        double sumVJ = 0;
        double sumVU = 0;
        int cnt = 0;
        int cntJ = 0;
        int cntU = 0;
        for (const auto& car: graph.cars()) {
            if (car->update()) {
                sumVU += car->v();
                cntU++;
            } else if (car->jams()) {
                sumVJ += car->v();
                cntJ++;
            } else {
                sumV += car->v();
                cnt++;
            }
        }
        plot.setPoint(0, cars, sumV / (cnt + 0.01));
        plot.setPoint(1, cars, sumVJ / (cntJ + 0.01));
        plot.setPoint(2, cars, sumVU / (cntU + 0.01));

        plot.draw();
    }

    return 0;
}

