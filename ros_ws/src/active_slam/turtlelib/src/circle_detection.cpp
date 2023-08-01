#include "turtlelib/circle_detection.hpp"

namespace turtlelib
{
Point compute_centroid(std::vector<Point> cluster)
{
  int N = cluster.size();
  double x_sum = 0.0;
  double y_sum = 0.0;
  for (int i = 0; i < N; i++) {
    x_sum += cluster.at(i).x;
    y_sum += cluster.at(i).y;
  }
  return {x_sum / N, y_sum / N};
}

double compute_mean_z(std::vector<Point> cluster_shifted)
{
  int N = cluster_shifted.size();
  double z_sum = 0.0;
  for (int i = 0; i < N; i++) {
    z_sum += cluster_shifted.at(i).x * cluster_shifted.at(i).x + cluster_shifted.at(i).y *
      cluster_shifted.at(i).y;
  }
  return z_sum / N;
}

arma::mat create_z_matrix(std::vector<Point> cluster_shifted)
{
  int N = cluster_shifted.size();
  arma::mat Z = arma::mat(N, 4, arma::fill::ones);
  for (int i = 0; i < N; i++) {
    Z(
      i,
      0) = cluster_shifted.at(i).x * cluster_shifted.at(i).x + cluster_shifted.at(i).y *
      cluster_shifted.at(i).y;
    Z(i, 1) = cluster_shifted.at(i).x;
    Z(i, 2) = cluster_shifted.at(i).y;
  }
  return Z;
}

Circle find_center_dim(arma::vec A)
{
  double a = -A(1) / (2 * A(0));
  double b = -A(2) / (2 * A(0));
  double r = std::sqrt((A(1) * A(1) + A(2) * A(2) - 4 * A(0) * A(3)) / (4 * A(0) * A(0)));
  return {{a, b}, r};
}

Circle find_circle(std::vector<Point> cluster)
{
  Point p = compute_centroid(cluster);
  std::cout << "centroid x: " << p.x << std::endl;
  std::cout << "centroid y: " << p.y << std::endl;
  int N = cluster.size();
  std::vector<Point> cluster_shifted {};
  for (int i = 0; i < N; i++) {
    cluster_shifted.push_back({(cluster.at(i).x - p.x), (cluster.at(i).y - p.y)});
  }
  double z_bar = compute_mean_z(cluster_shifted);
  std::cout << "z_bar: " << z_bar << std::endl;
  arma::mat Z = create_z_matrix(cluster_shifted);
  std::cout << "Z: \n" << Z << std::endl;
  arma::mat M = (1 / N) * Z.t() * Z;
  arma::mat H_inv = arma::mat(4, 4, arma::fill::zeros);
  H_inv(3, 3) = -2 * z_bar;
  H_inv(0, 3) = 0.5;
  H_inv(1, 1) = 1;
  H_inv(2, 2) = 1;
  H_inv(3, 0) = 0.5;
  std::cout << "H_inv" << H_inv << std::endl;

  arma::mat U;
  arma::vec sigma;
  arma::mat V;
  arma::svd(U, sigma, V, Z);
  arma::vec A;
  if (sigma(3) < 10e-12) {
    A = V.col(3);
  } else {
    arma::mat Sigma_bar = arma::diagmat(sigma);
    std::cout << "Sigma_bar" << Sigma_bar << std::endl;
    arma::mat Y = V * Sigma_bar * V.t();
    std::cout << "V: \n" << V << std::endl;
    std::cout << "Y" << Y << std::endl;
    arma::mat Q = Y * H_inv * Y;
    arma::cx_vec eigen_val;
    arma::cx_mat eigen_vec;
    std::cout << "Q" << Q << std::endl;
    arma::eig_gen(eigen_val, eigen_vec, Q);
    arma::vec e_val = arma::vec(4);
    std::cout << "e val: \n" << e_val << std::endl;
    int m = e_val.size();
    double min_e_val = 999999;
    double index_min = 0;
    for (int i = 0; i < m; i++) {
      if (eigen_val(i).real() > 0.0 && eigen_val(i).real() < min_e_val) {
        min_e_val = eigen_val(i).real();
        index_min = i;
      }
    }
    arma::vec A_star = arma::vec(4);
    for (int b = 0; b < 4; b++) {
      A_star.at(b) = eigen_vec(b, index_min).real();
    }
    std::cout << "A_star: \n" << A_star << std::endl;
    A = Y.i() * A_star;
    std::cout << "A: \n" << A << std::endl;
  }
  Circle circ = find_center_dim(A);
  circ.center.x += p.x;
  circ.center.y += p.y;
  return circ;
}
}
