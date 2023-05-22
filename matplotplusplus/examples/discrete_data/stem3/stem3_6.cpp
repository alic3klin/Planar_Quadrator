#include <cmath>
#include <matplot/matplot.h>

int main() {
    using namespace matplot;
    std::vector<double> x = linspace(-pi, pi, 40);
    std::vector<double> z = transform(x, [](double x) { return cos(x); });

    stem3(z, "filled");

    show();
    return 0;
}