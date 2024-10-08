#pragma once


    double trapz(std::array<double, 2> x, std::array<double, 2> f) {
        return (f[1] + f[0]) * (x[1] - x[0]) / 2.0;
    }