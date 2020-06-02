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
    struct Edge {
        int from, to;
        std::vector<sf::Vertex> line;
        RoadOptions options;
    };
    Graph(int sx, int sy): sx_(sx), sy_(sy), window_(sx, sy, "Graph") {}
    void draw() {
        window_.window().clear();
        for (const auto& pair: edges_) {
            for (const auto& pair2: pair.second) {
                const auto& edge = pair2.second;
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
    Algorithm(const Graph& graph): graph_(graph) {}
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
                int newD = infos[curv].d + pair.second.options.length / pair.second.options.vmax * FACTOR;
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
        return result;
    };
protected:
    const Graph& graph_;
};

class Car : public BaseCar {
static const constexpr double VFACTOR = 0.3;
static const constexpr int RADIUS = 3;
public:
    Car(const Graph& graph, int from, int to): graph_(graph) {
        Algorithm algo(graph_);
        path_ = algo.getPath(from, to);
        index_ = 0;
        edgePart_ = 0;
    }

    void process(const sf::Time& elapsed) {
        if (index_ >= path_.size() - 1) {
            return;
        }
        int from = path_[index_];
        int to = path_[index_ + 1];
        const auto& edge = graph_.edges().at(from).at(to);
        double v = VFACTOR * edge.options.vmax;
        edgePart_ += v * elapsed.asSeconds();
        if (edgePart_ > edge.options.length) {
            index_++;
            edgePart_ = 0;
        }
    }

    void draw(sf::RenderWindow& window) {
        for (const auto& id: path_) { // {path_.front(), path_.back()}) {
            sf::CircleShape shape(RADIUS);
            auto start = graph_.vertices().at(id).position;
            shape.setPosition(start.x - RADIUS, start.y - RADIUS);
            shape.setFillColor(sf::Color::Red);
            window.draw(shape);
        }
        if (index_ >= path_.size() - 1) {
            return;
        }
        {
        sf::Vertex position = getPosition();
        sf::CircleShape shape(RADIUS);
        shape.setPosition(position.position.x - RADIUS, position.position.y - RADIUS);
        shape.setFillColor(sf::Color::Green);
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

private:
    const Graph& graph_;
    std::vector<int> path_;
    int index_;
    double edgePart_;
};

class CarGenerator {
public:
    CarGenerator(const Graph& graph) : graph_(graph), gen_(time(0)), dist_(0, graph.vertices().size() - 1) {
        ids_.reserve(graph.vertices().size());
        for (const auto& v: graph.vertices()) {
            ids_.push_back(v.first);
        }
    }
    std::unique_ptr<Car> generate() {
        return std::make_unique<Car>(graph_, randomVertex(), randomVertex());
    }
    int randomVertex() {
        return ids_[dist_(gen_)];
    }
private:
    const Graph& graph_;
    std::vector<int> ids_;
    std::mt19937 gen_;
    std::uniform_int_distribution<int> dist_;
};

class OsmHandler {
    std::map<std::string, Graph::RoadOptions> OPTIONS{
        {"motorway", {110, 3}},
        {"motorway_link", {90, 3}},
        {"trunk", {90, 2}},
        {"trunk_link", {90, 2}},
        {"primary", {60, 2}},
        {"primary_link", {60, 2}},
        {"secondary", {60, 2}},
        {"secondary_link", {60, 1}},
        {"tertiary", {45, 1}},
        {"tertiary_link", {45, 1}},
        {"unclassified", {30, 1}},
        //{"residential", {}}//,
        //{"service", {BASE_WIDTH, RoadType::SIDE}}
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
    static const int CARS = 3;

    Graph graph0 = load();
    GraphCompacter compacter(graph0);
    Graph graph = compacter.run(g_rootId);

    std::cout << "Initial graph =" << graph0.vertices().size() << std::endl;
    std::cout << "Reduced graph =" << graph.vertices().size() << std::endl;

    CarGenerator generator(graph);

    int x;
    std::cin >> x;
    std::cout << "Start!";

    sf::Clock clock;
    sf::Time totalTime;
    while (g_running)
    {
        sf::Time elapsed = clock.restart();
        totalTime += elapsed;

        while (graph.carsCount() < CARS) {
            graph.addCar(generator.generate());
        }

        graph.process(elapsed);
        graph.draw();
    }

    return 0;
}

