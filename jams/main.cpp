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

class Graph {
public:
    struct RoadOptions {};
    struct Edge {
        int from, to;
        std::vector<sf::Vertex> line;
        RoadOptions options;
    };
    Graph(int sx, int sy): window_(sx, sy, "Graph") {}
    void draw() {
        window_.window().clear();
        for (const auto& pair: edges_) {
            for (const auto& pair2: pair.second) {
                for (const auto& edge: pair2.second) {
                    window_.window().draw(edge.line.data(), edge.line.size(), sf::LineStrip);
                }
            }
        }
        window_.window().display();
    }
    void process() {
        window_.process();
    }
private:
    void addEdge(int u, int v, std::vector<sf::Vertex> line, RoadOptions options) {
        vertices_.insert(u);
        vertices_.insert(v);
        edges_[u][v].push_back({u, v, line, options});
        std::reverse(line.begin(), line.end());
        edges_[v][u].push_back({v, u, line, options});
    }
    friend class OsmHandler;
    std::map<int, std::map<int, std::vector<Edge>>> edges_;
    std::set<int> vertices_;
    Window window_;
};

class OsmHandler {
    std::map<std::string, Graph::RoadOptions> OPTIONS{
        {"motorway", {}},
        {"motorway_link", {}},
        {"trunk", {}},
        {"trunk_link", {}},
        {"primary", {}},
        {"primary_link", {}},
        {"secondary", {}},
        {"secondary_link", {}},
        {"tertiary", {}},
        {"tertiary_link", {}},
        {"unclassified", {}},
        //{"residential", {}}//,
        //{"service", {BASE_WIDTH, RoadType::SIDE}}
        {"foo", {}}
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
    sf::Clock clock;
    sf::Time totalTime;

    Graph graph = load();

    while (g_running)
    {
        sf::Time elapsed = clock.restart();
        totalTime += elapsed;

        graph.draw();
        graph.process();
        /*
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
        */
    }

    return 0;
}

