#ifndef PLOTTER_HPP
#define PLOTTER_HPP

#include <iostream>
#include <vector>
#include <string>
#include <cstdio>
#include <cmath>
#include "structures/geo/geoUtils.hpp"

struct PathEntry {
    PointsList  path;
    std::string title;
    std::string color;
    bool        isScatter;  // true = punti senza linee, false = linespoints
};

struct CylinderPlotData {
    double x, y, r, hBase, hTop;
};

class Plotter {
private:
    FILE* gnuplotPipe;
    std::vector<PathEntry>        pendingPaths;
    std::vector<CylinderPlotData> cylinders;

    FILE* openPipe() {
#ifdef _WIN32
        return _popen("gnuplot -persist", "w");
#else
        return popen("gnuplot -persist", "w");
#endif
    }
    void closePipe(FILE* p) {
        if (!p) return;
#ifdef _WIN32
        _pclose(p);
#else
        pclose(p);
#endif
    }

public:
    Plotter() : gnuplotPipe(nullptr) {
        gnuplotPipe = openPipe();
        if (!gnuplotPipe) {
            std::cerr << "Errore: Gnuplot non trovato!\n";
            return;
        }
        send("set mouse");
        send("set grid");
        send("set view 60, 30");
        send("set xlabel 'X (m)'");
        send("set ylabel 'Y (m)'");
        send("set zlabel 'Z (m)'");
        send("set parametric");
        send("set urange [0:2*pi]");
        send("set style data linespoints");
        send("unset colorbox");
        send("set hidden3d");
    }

    ~Plotter() { closePipe(gnuplotPipe); }

    void send(const std::string& command) {
        if (!gnuplotPipe) return;
        fprintf(gnuplotPipe, "%s\n", command.c_str());
        fflush(gnuplotPipe);
    }

    // Accoda una path connessa (linespoints)
    void addPath(const PointsList& path,
                 const std::string& title = "UAV Path",
                 const std::string& color = "blue") {
        if (path.size() == 0) return;
        pendingPaths.push_back({path, title, color, false});
    }

    // Accoda punti senza linee di collegamento (punti CSV, centroidi, ecc.)
    void addScatter(const PointsList& points,
                    const std::string& title = "Points",
                    const std::string& color = "gray") {
        if (points.size() == 0) return;
        pendingPaths.push_back({points, title, color, true});
    }

    void addCylinder(double cx, double cy, double r,
                     double zBase = 0.0, double zTop = 180.0) {
        cylinders.push_back({cx, cy, r, zBase, zTop});
    }

    void setVRange(double zMin, double zMax) {
        char buf[128];
        snprintf(buf, sizeof(buf), "set vrange [%.2f:%.2f]", zMin, zMax);
        send(buf);
    }

    // Unico splot: cilindri wireframe rossi + scatter CSV grigi
    //              + centroidi neri + path DRSTASA blu
    void flush() {
        if (!gnuplotPipe) return;
        if (pendingPaths.empty() && cylinders.empty()) return;

        std::string splotCmd = "splot ";
        bool first = true;

        // ── Cilindri ─────────────────────────────────────────────────────────
        for (size_t i = 0; i < cylinders.size(); ++i) {
            const auto& c = cylinders[i];
            if (!first) splotCmd += ", ";
            first = false;

            char buf[512];
            snprintf(buf, sizeof(buf),
                "%.4f+%.4f*cos(u), %.4f+%.4f*sin(u), v "
                "with lines lc rgb 'red' lw 1 notitle",
                c.x, c.r, c.y, c.r);
            splotCmd += buf;

            // Disco superiore e inferiore (dati inline)
            splotCmd += ", '-' with lines lc rgb 'red' lw 1 notitle";
            splotCmd += ", '-' with lines lc rgb 'red' lw 1 notitle";
        }

        // ── Path / scatter ───────────────────────────────────────────────────
        for (size_t i = 0; i < pendingPaths.size(); ++i) {
            if (!first) splotCmd += ", ";
            first = false;
            const auto& pe = pendingPaths[i];
            char buf[256];

            if (pe.isScatter) {
                // Centroidi: pallini neri più grandi; punti CSV: pallini grigi piccoli
                int ps = (pe.color == "black") ? 2 : 1;
                snprintf(buf, sizeof(buf),
                    "'-' with points pt 7 ps %d lc rgb '%s' title '%s'",
                    ps, pe.color.c_str(), pe.title.c_str());
            } else {
                snprintf(buf, sizeof(buf),
                    "'-' with linespoints lw 2.5 lc rgb '%s' pt 7 ps 1.2 title '%s'",
                    pe.color.c_str(), pe.title.c_str());
            }
            splotCmd += buf;
        }

        fprintf(gnuplotPipe, "%s\n", splotCmd.c_str());
        fflush(gnuplotPipe);

        // ── Dati inline: dischi dei cilindri ─────────────────────────────────
        const int N = 36;
        for (const auto& c : cylinders) {
            for (int k = 0; k <= N; ++k) {
                double a = 2.0 * M_PI * k / N;
                fprintf(gnuplotPipe, "%f %f %f\n",
                    c.x + c.r*cos(a), c.y + c.r*sin(a), c.hTop);
            }
            fprintf(gnuplotPipe, "e\n");
            for (int k = 0; k <= N; ++k) {
                double a = 2.0 * M_PI * k / N;
                fprintf(gnuplotPipe, "%f %f %f\n",
                    c.x + c.r*cos(a), c.y + c.r*sin(a), c.hBase);
            }
            fprintf(gnuplotPipe, "e\n");
        }

        // ── Dati inline: path e scatter ───────────────────────────────────────
        for (const auto& pe : pendingPaths) {
            for (int j = 0; j < (int)pe.path.size(); ++j) {
                fprintf(gnuplotPipe, "%f %f %f\n",
                    pe.path.getX(j), pe.path.getY(j), pe.path.getZ(j));
            }
            fprintf(gnuplotPipe, "e\n");
        }
        fflush(gnuplotPipe);
    }

    void clearPaths()     { pendingPaths.clear(); }
    void clearCylinders() { cylinders.clear(); }
};

#endif // PLOTTER_HPP