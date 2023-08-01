//Contributors: Katie Hughes, Rintaroh Shima, and Marno Nel

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <iostream>
#include "turtlelib/rigid2d.hpp"
#include "turtlelib/diff_drive.hpp"
#include <sstream>
#include "turtlelib/circle_detection.hpp"
#include "turtlelib/discriminant.hpp"

TEST_CASE("Twist2D Operator()", "[transform]") { // Rintaroh, Shima (edited to access private vars)
   turtlelib::Vector2D v;
   v.x = 5;
   v.y = 6;
   double theta = turtlelib::PI/4;
   turtlelib::Transform2D trans = turtlelib::Transform2D(v, theta);
   turtlelib::Twist2D t = {3, 4, 5};
   turtlelib::Twist2D twist = trans(t);
   REQUIRE_THAT(twist.getW(), Catch::Matchers::WithinAbs(3, 1e-5));
   REQUIRE_THAT(twist.getX(), Catch::Matchers::WithinAbs(17.2929, 1e-5));
   REQUIRE_THAT(twist.getY(), Catch::Matchers::WithinAbs(-8.63604, 1e-5));
}

TEST_CASE( "Inverse", "[transform]" ) { // Katie, Hughes
   float my_x = 0.;
   float my_y = 1.;
   float my_ang = turtlelib::PI/2;
   turtlelib::Transform2D Ttest = {turtlelib::Vector2D{my_x,my_y}, my_ang};
   turtlelib::Transform2D Ttest_inv = Ttest.inv();
   REQUIRE( (Ttest.inv()).rotation() == -my_ang);
   REQUIRE_THAT(Ttest_inv.translation().x, Catch::Matchers::WithinAbs(-1.0, 1e-5));
   REQUIRE_THAT(Ttest_inv.translation().y, Catch::Matchers::WithinAbs(0.0, 1e-5));
}

TEST_CASE("operator *=", "[transform]"){    //Megan, Sindelar
    turtlelib::Vector2D trans_ab = {1,2};
    double rotate_ab = 0;
    turtlelib::Transform2D T_ab_1 = {trans_ab, rotate_ab};      //T_ab's are all the same,
    turtlelib::Transform2D T_ab_2 = {trans_ab, rotate_ab};      //but, need different vars
    turtlelib::Transform2D T_ab_3 = {trans_ab, rotate_ab};      //b/c getting overwritten otherwise
    turtlelib::Vector2D trans_bc = {3,4};
    double rotate_bc = turtlelib::PI/2;
    turtlelib::Transform2D T_bc = {trans_bc, rotate_bc};

    REQUIRE_THAT((T_ab_1*=T_bc).translation().x, Catch::Matchers::WithinAbs(4.0, 1e-5));
    REQUIRE_THAT((T_ab_2*=T_bc).translation().y, Catch::Matchers::WithinAbs(6.0, 1e-5));
    REQUIRE_THAT((T_ab_3*=T_bc).rotation(), Catch::Matchers::WithinAbs((turtlelib::PI/2), 1e-5));
}

TEST_CASE("Rotation and Translation", "[transform]"){   //Megan Sindelar
    turtlelib::Vector2D trans = {1,2};
    double rotate = 90;
    turtlelib::Transform2D T = {trans, rotate};
    REQUIRE_THAT(T.translation().x, Catch::Matchers::WithinAbs(trans.x, 1e-5));
    REQUIRE_THAT(T.translation().y, Catch::Matchers::WithinAbs(trans.y, 1e-5));
    REQUIRE_THAT(T.rotation(), Catch::Matchers::WithinAbs(rotate, 1e-5));
}

TEST_CASE("Output stream operator <<", "[transform]"){    //Megan, Sindelar
    std::stringstream ss;                                   //create an object of stringstream
    std::streambuf* old_cout_streambuf = std::cout.rdbuf();     //pointer that holds the output
    std::cout.rdbuf(ss.rdbuf());        //redirect output stream to a stringstream
    turtlelib::Transform2D T_m = {{4, 3}, 0};
    std::cout << T_m << std::endl;
    std::cout.rdbuf(old_cout_streambuf);    //restore the original stream buffer

    REQUIRE(ss.str() == "deg: 0 x: 4 y: 3\n");
}

//reference chat gpt https://chat.openai.com/
TEST_CASE("Input stream operator >>, [transform]"){     //Megan Sindelar
    std::stringstream ss("deg: 0 x: 4 y: 5");            //create an object of string stream
    std::streambuf* old_cin_streambuf = std::cin.rdbuf();   //pointer that holds the input
    std::cin.rdbuf(ss.rdbuf());                     //redirect input stream to a stringstream
    std::string input1, input2, input3, input4, input5, input6;     // variable to store input
    std::cin >> input1;                              //user input (before whitespace)
    std::cin >> input2;                              //user input (before whitespace)
    std::cin >> input3;                              //user input (before whitespace)
    std::cin >> input4;                              //user input (before whitespace)
    std::cin >> input5;                              //user input (before whitespace)
    std::cin >> input6;                              //user input (before whitespace)
    std::cin.rdbuf(old_cin_streambuf);              //restore the original stream buffer

    std::string input;
    input = input1 + " " + input2 + " " + input3 + " " + input4 + " " + input5 + " " + input6;

    REQUIRE(input == "deg: 0 x: 4 y: 5");
}

TEST_CASE("operator()(Twist2D t)","[transform]") // Marno, Nel (edited to access private vars)
{
    double test_rot = turtlelib::PI/2.0;
    double test_x = 0.0;
    double test_y = 1.0;
    turtlelib::Transform2D T_ab{{test_x,test_y}, test_rot};
    turtlelib::Twist2D V_b{1, 1, 1};
    turtlelib::Twist2D V_a = T_ab(V_b);
    REQUIRE_THAT(V_a.getW(), Catch::Matchers::WithinAbs(1.0, 1e-5));
    REQUIRE_THAT(V_a.getX(), Catch::Matchers::WithinAbs(0.0, 1e-5));
    REQUIRE_THAT(V_a.getY(), Catch::Matchers::WithinAbs(1.0, 1e-5));
}

TEST_CASE("normalize rad","[vector]") // Megan Sindelar
{
    double rad1 = turtlelib::normalize_angle(turtlelib::PI);
    double rad2 = turtlelib::normalize_angle(-turtlelib::PI);
    double rad3 = turtlelib::normalize_angle(0.0);
    double rad4 = turtlelib::normalize_angle(-turtlelib::PI/4);
    double rad5 = turtlelib::normalize_angle(3*turtlelib::PI/2);
    double rad6 = turtlelib::normalize_angle(-5*turtlelib::PI/2);

    REQUIRE_THAT(rad1, Catch::Matchers::WithinAbs(turtlelib::PI, 1e-5));
    REQUIRE_THAT(rad2, Catch::Matchers::WithinAbs(turtlelib::PI, 1e-5));
    REQUIRE_THAT(rad3, Catch::Matchers::WithinAbs(0.0, 1e-5));
    REQUIRE_THAT(rad4, Catch::Matchers::WithinAbs(-turtlelib::PI/4, 1e-5));
    REQUIRE_THAT(rad5, Catch::Matchers::WithinAbs(-1.553, 1e-3));
    REQUIRE_THAT(rad6, Catch::Matchers::WithinAbs(-turtlelib::PI/2, 1e-5));  
}

TEST_CASE("operator+=","[vector]") // Megan Sindelar
{
    turtlelib::Vector2D vec1{5.0, 3.0};
    turtlelib::Vector2D vec2{4.0, 2.0};

    vec1+=vec2;

    REQUIRE_THAT(vec1.x, Catch::Matchers::WithinAbs(9.0, 1e-5));
    REQUIRE_THAT(vec1.y, Catch::Matchers::WithinAbs(5.0, 1e-5));
}

TEST_CASE("operator+","[vector]") // Megan Sindelar
{
    turtlelib::Vector2D vec1{5.0, 3.0};
    turtlelib::Vector2D vec2{4.0, 2.0};
    turtlelib::Vector2D vec3 = vec1 + vec2;

    REQUIRE_THAT(vec3.x, Catch::Matchers::WithinAbs(9.0, 1e-5));
    REQUIRE_THAT(vec3.y, Catch::Matchers::WithinAbs(5.0, 1e-5));
}

TEST_CASE("operator-=","[vector]") // Megan Sindelar
{
    turtlelib::Vector2D vec1{5.0, 3.0};
    turtlelib::Vector2D vec2{4.0, 2.0};

    vec1-=vec2;

    REQUIRE_THAT(vec1.x, Catch::Matchers::WithinAbs(1.0, 1e-5));
    REQUIRE_THAT(vec1.y, Catch::Matchers::WithinAbs(1.0, 1e-5));
}

TEST_CASE("operator-","[vector]") // Megan Sindelar
{
    turtlelib::Vector2D vec1{5.0, 3.0};
    turtlelib::Vector2D vec2{4.0, 2.0};
    turtlelib::Vector2D vec3 = vec1 - vec2;

    REQUIRE_THAT(vec3.x, Catch::Matchers::WithinAbs(1.0, 1e-5));
    REQUIRE_THAT(vec3.y, Catch::Matchers::WithinAbs(1.0, 1e-5));
}

TEST_CASE("operator*=","[vector]") // Megan Sindelar
{
    turtlelib::Vector2D vec1{5.0, 3.0};
    turtlelib::Vector2D vec2{4.0, 2.0};

    vec1*=vec2;

    REQUIRE_THAT(vec1.x, Catch::Matchers::WithinAbs(20.0, 1e-5));
    REQUIRE_THAT(vec1.y, Catch::Matchers::WithinAbs(6.0, 1e-5));
}

TEST_CASE("operator*","[vector]") // Megan Sindelar
{
    turtlelib::Vector2D vec1{5.0, 3.0};
    turtlelib::Vector2D vec2{4.0, 2.0};
    turtlelib::Vector2D vec3 = vec1 * vec2;

    REQUIRE_THAT(vec3.x, Catch::Matchers::WithinAbs(20.0, 1e-5));
    REQUIRE_THAT(vec3.y, Catch::Matchers::WithinAbs(6.0, 1e-5));
}

TEST_CASE("find dot product","[vector]") // Megan Sindelar
{
    turtlelib::Vector2D vec1{5.0, 3.0};
    turtlelib::Vector2D vec2{4.0, 2.0};

    double dp = dot(vec1, vec2);

    REQUIRE_THAT(dp, Catch::Matchers::WithinAbs(26, 1e-5));
}

TEST_CASE("find magnitude","[vector]") // Megan Sindelar
{
    turtlelib::Vector2D vec1{5.0, 3.0};

    double mag = magnitude(vec1);

    REQUIRE_THAT(mag, Catch::Matchers::WithinAbs(5.83095, 1e-5));  
}

TEST_CASE("find angle","[vector]") // Megan Sindelar
{
    turtlelib::Vector2D vec1{5.0, 3.0};
    turtlelib::Vector2D vec2{4.0, 2.0};

    double ang = angle(vec1, vec2);

    REQUIRE_THAT(ang, Catch::Matchers::WithinAbs(0.07677, 1e-5));  
}

TEST_CASE("Transform following a constant twist ","[transform]") // Megan Sindelar
{
    turtlelib::Twist2D twist = {-1.24, -2.15, -2.92};

    turtlelib::Transform2D T = integrate_twist(twist);

    REQUIRE_THAT(T.translation().x, Catch::Matchers::WithinAbs(-3.229863264722, 1e-5)); 
    REQUIRE_THAT(T.translation().y, Catch::Matchers::WithinAbs(-1.05645265317421, 1e-5)); 
    REQUIRE_THAT(T.rotation(), Catch::Matchers::WithinAbs(-1.24, 1e-5));

    turtlelib::Twist2D twist_trans = {0.0, 1.0, 1.0};

    turtlelib::Transform2D T_trans = integrate_twist(twist_trans);

    REQUIRE_THAT(T_trans.translation().x, Catch::Matchers::WithinAbs(1.0, 1e-5)); 
    REQUIRE_THAT(T_trans.translation().y, Catch::Matchers::WithinAbs(1.0, 1e-5)); 
    REQUIRE_THAT(T_trans.rotation(), Catch::Matchers::WithinAbs(0.0, 1e-5));

    turtlelib::Twist2D twist_rot = {1.0, 0.0, 0.0};

    turtlelib::Transform2D T_rot = integrate_twist(twist_rot);

    REQUIRE_THAT(T_rot.translation().x, Catch::Matchers::WithinAbs(0.0, 1e-5)); 
    REQUIRE_THAT(T_rot.translation().y, Catch::Matchers::WithinAbs(0.0, 1e-5)); 
    REQUIRE_THAT(T_rot.rotation(), Catch::Matchers::WithinAbs(1.0, 1e-5));
}

TEST_CASE("forward drive","[diffdrive]") // Megan Sindelar
{
    double rad = 1.0;
    double track = 4.0;
    turtlelib::Phi p = {1.0, 1.0};
    turtlelib::Twist2D t = {0.0, 1.0, 0.0};
    turtlelib::DiffDrive rob = {track, rad};
    turtlelib::DiffDrive rob1 = {track, rad};
    turtlelib::Phi phi = rob.Inverse_Kin(t);
    rob1.Forward_Kin(p);

    REQUIRE_THAT(phi.r, Catch::Matchers::WithinAbs(1.0, 1e-5));
    REQUIRE_THAT(phi.l, Catch::Matchers::WithinAbs(1.0, 1e-5)); 
    REQUIRE_THAT(rob1.theta_get(), Catch::Matchers::WithinAbs(0.0, 1e-5));
    REQUIRE_THAT(rob1.x_get(), Catch::Matchers::WithinAbs(1.0, 1e-5));
    REQUIRE_THAT(rob1.y_get(), Catch::Matchers::WithinAbs(0.0, 1e-5));
}

TEST_CASE("backwards drive","[diffdrive]") // Megan Sindelar
{
    double rad = 1.0;
    double track = 4.0;
    turtlelib::Phi p = {-1.0, -1.0};
    turtlelib::Twist2D t = {0.0, -1.0, 0.0};
    turtlelib::DiffDrive rob = {track, rad};
    turtlelib::DiffDrive rob1 = {track, rad};
    turtlelib::Phi phi = rob.Inverse_Kin(t);
    rob1.Forward_Kin(p);

    REQUIRE_THAT(phi.r, Catch::Matchers::WithinAbs(-1.0, 1e-5));
    REQUIRE_THAT(phi.l, Catch::Matchers::WithinAbs(-1.0, 1e-5)); 
    REQUIRE_THAT(rob1.theta_get(), Catch::Matchers::WithinAbs(0.0, 1e-5));
    REQUIRE_THAT(rob1.x_get(), Catch::Matchers::WithinAbs(-1.0, 1e-5));
    REQUIRE_THAT(rob1.y_get(), Catch::Matchers::WithinAbs(0.0, 1e-5));
}

TEST_CASE("rotation CW drive","[diffdrive]") // Megan Sindelar
{
    double rad = 1.0;
    double track = 4.0;
    turtlelib::Phi p = {-1.0, 1.0};
    turtlelib::Twist2D t = {1.0, 0.0, 0.0};
    turtlelib::DiffDrive rob = {track, rad};
    turtlelib::DiffDrive rob1 = {track, rad};
    turtlelib::Phi phi = rob.Inverse_Kin(t);
    rob1.Forward_Kin(p);

    REQUIRE_THAT(phi.r, Catch::Matchers::WithinAbs(2.0, 1e-5));
    REQUIRE_THAT(phi.l, Catch::Matchers::WithinAbs(-2.0, 1e-5)); 
    REQUIRE_THAT(rob1.theta_get(), Catch::Matchers::WithinAbs(-0.5, 1e-5));
    REQUIRE_THAT(rob1.x_get(), Catch::Matchers::WithinAbs(0.0, 1e-5));
    REQUIRE_THAT(rob1.y_get(), Catch::Matchers::WithinAbs(0.0, 1e-5));
}

TEST_CASE("rotation CCW drive","[diffdrive]") // Megan Sindelar
{
    double rad = 1.0;
    double track = 4.0;
    turtlelib::Phi p = {1.0, -1.0};
    turtlelib::Twist2D t = {-1.0, 0.0, 0.0};
    turtlelib::DiffDrive rob = {track, rad};
    turtlelib::DiffDrive rob1 = {track, rad};
    turtlelib::Phi phi = rob.Inverse_Kin(t);
    rob1.Forward_Kin(p);

    REQUIRE_THAT(phi.r, Catch::Matchers::WithinAbs(-2.0, 1e-5));
    REQUIRE_THAT(phi.l, Catch::Matchers::WithinAbs(2.0, 1e-5)); 
    REQUIRE_THAT(rob1.theta_get(), Catch::Matchers::WithinAbs(0.5, 1e-5));
    REQUIRE_THAT(rob1.x_get(), Catch::Matchers::WithinAbs(0.0, 1e-5));
    REQUIRE_THAT(rob1.y_get(), Catch::Matchers::WithinAbs(0.0, 1e-5));
}

TEST_CASE("arc CW drive","[diffdrive]") // Megan Sindelar
{
    double rad = 1.0;
    double track = 4.0;
    turtlelib::Phi p = {1.0, 2.0};
    turtlelib::Twist2D t = {1.0, 1.0, 0.0}; 
    turtlelib::DiffDrive rob = {track, rad};
    turtlelib::DiffDrive rob1 = {track, rad};
    turtlelib::Phi phi = rob.Inverse_Kin(t);
    rob1.Forward_Kin(p);

    REQUIRE_THAT(phi.r, Catch::Matchers::WithinAbs(3.0, 1e-5));
    REQUIRE_THAT(phi.l, Catch::Matchers::WithinAbs(-1.0, 1e-5)); 
    REQUIRE_THAT(rob1.theta_get(), Catch::Matchers::WithinAbs(-0.25, 1e-5));
    REQUIRE_THAT(rob1.x_get(), Catch::Matchers::WithinAbs(1.48442, 1e-5));
    REQUIRE_THAT(rob1.y_get(), Catch::Matchers::WithinAbs(-0.186525, 1e-5));
}

TEST_CASE("arc CCW drive","[diffdrive]") // Megan Sindelar
{
    double rad = 1.0;
    double track = 4.0;
    turtlelib::Phi p = {2.0, 1.0};
    turtlelib::Twist2D t = {-1.0, 1.0, 0.0};
    turtlelib::DiffDrive rob = {track, rad};
    turtlelib::DiffDrive rob1 = {track, rad};
    turtlelib::Phi phi = rob.Inverse_Kin(t);
    rob1.Forward_Kin(p);

    REQUIRE_THAT(phi.r, Catch::Matchers::WithinAbs(-1.0, 1e-5));
    REQUIRE_THAT(phi.l, Catch::Matchers::WithinAbs(3.0, 1e-5)); 
    REQUIRE_THAT(rob1.theta_get(), Catch::Matchers::WithinAbs(0.25, 1e-5));
    REQUIRE_THAT(rob1.x_get(), Catch::Matchers::WithinAbs(1.48442, 1e-5));
    REQUIRE_THAT(rob1.y_get(), Catch::Matchers::WithinAbs(0.186525, 1e-5));
}


TEST_CASE("arc of a circle", "[diffdrive]") { //Nick Morales
    turtlelib::DiffDrive robot {0.16, 0.033};
    turtlelib::Phi wheels {3.5, 5.0};
    SECTION("forward kinematics") {
        //Update configuration using new wheel position
        robot.Forward_Kin(wheels);
        //Check that configuration matches expected configuration
        REQUIRE_THAT(robot.x_get(), 
            Catch::Matchers::WithinAbs(0.138023393683395, 1e-5));
        REQUIRE_THAT(robot.y_get(), 
            Catch::Matchers::WithinAbs(-0.0215224326983102, 1e-5));
        REQUIRE_THAT(robot.theta_get(), 
            Catch::Matchers::WithinAbs(-0.309375, 1e-5));
        REQUIRE_THAT(robot.phi_r_get(), 
            Catch::Matchers::WithinAbs(3.5, 1e-5));
        REQUIRE_THAT(robot.phi_l_get(), 
            Catch::Matchers::WithinAbs(5.0, 1e-5));
    }
}

TEST_CASE("compute centroid", "[circle_detection]") { //Megan Sindelar
    std::vector<turtlelib::Point> cluster = {{1.0,7.0}, {2.0, 6.0}, {5.0, 8.0}, {7.0, 7.0}, {9.0, 5.0}, {3.0, 7.0}};
    std::vector<turtlelib::Point> cluster2 = {{-1,0},{-0.3,-0.06},{0.3,0.1},{1,0}};
    turtlelib::Point point = turtlelib::compute_centroid(cluster);
    turtlelib::Point point2 = turtlelib::compute_centroid(cluster2);

    REQUIRE_THAT(point.x, Catch::Matchers::WithinAbs(4.5, 1e-5));
    REQUIRE_THAT(point.y, Catch::Matchers::WithinAbs(6.666667, 1e-5));
    REQUIRE_THAT(point2.x, Catch::Matchers::WithinAbs(0, 1e-5));
    REQUIRE_THAT(point2.y, Catch::Matchers::WithinAbs(0.01, 1e-5));
}

TEST_CASE("throw exception","[diffdrive]") // Megan Sindelar
{
    turtlelib::Twist2D q = {1.0, 1.0, 1.0};
    turtlelib::DiffDrive rob = {1.0, 1.0, {0.0, 0.0}, {0.0, 0.0, 0.0}};

    REQUIRE_THROWS_AS(rob.Inverse_Kin(q), std::logic_error);
}

TEST_CASE("test discriminant","[discriminant]") // Megan Sindelar
{
    turtlelib::Disc disc = {1.0, 1.0, 1.0};
    double dis = turtlelib::calc_disc(disc);

    REQUIRE_THAT(dis, Catch::Matchers::WithinAbs(-3.0, 1e-5));
}

TEST_CASE("compute mean of z", "[circle_detection]") { //Megan Sindelar
    std::vector<turtlelib::Point> cluster = {{1.0,7.0}, {2.0, 6.0}, {5.0, 8.0}, {7.0, 7.0}, {9.0, 5.0}, {3.0, 7.0}};
    std::vector<turtlelib::Point> cluster2 = {{-1,0},{-0.3,-0.06},{0.3,0.1},{1,0}};
    double mean = turtlelib::compute_mean_z(cluster);
    double mean2 = turtlelib::compute_mean_z(cluster2);

    REQUIRE_THAT(mean, Catch::Matchers::WithinAbs(73.5, 1e-5));
    REQUIRE_THAT(mean2, Catch::Matchers::WithinAbs(0.5484, 1e-5));
}

TEST_CASE("find size of Z", "[circle_detection]" ){ // Megan Sindelar
        std::vector<turtlelib::Point> cluster = {{1.0,7.0}, {2.0, 6.0}, {5.0, 8.0}, {7.0, 7.0}, {9.0, 5.0}, {3.0, 7.0}};
        std::vector<turtlelib::Point> cluster2 = {{-1,0},{-0.3,-0.06},{0.3,0.1},{1,0}};
        arma::mat Z= create_z_matrix(cluster);
        arma::mat Z2= create_z_matrix(cluster2);
        REQUIRE_THAT(Z.n_rows, Catch::Matchers::WithinAbs(6, 1e-5));
        REQUIRE_THAT(Z.n_cols, Catch::Matchers::WithinAbs(4, 1e-5));
        REQUIRE_THAT(Z2.n_rows, Catch::Matchers::WithinAbs(4, 1e-5));
        REQUIRE_THAT(Z2.n_cols, Catch::Matchers::WithinAbs(4, 1e-5));
}

TEST_CASE("find center and radius of circle", "[circle_detection]" ){ // Megan Sindelar
        std::vector<turtlelib::Point> cluster = {{1.0,7.0}, {2.0, 6.0}, {5.0, 8.0}, {7.0, 7.0}, {9.0, 5.0}, {3.0, 7.0}};
        std::vector<turtlelib::Point> cluster2 = {{-1,0},{-0.3,-0.06},{0.3,0.1},{1,0}};
        turtlelib::Circle c1 = find_circle(cluster);
        turtlelib::Circle c2 = find_circle(cluster2);
        REQUIRE_THAT(c1.center.x, Catch::Matchers::WithinAbs(4.615482, 10e-5));
        REQUIRE_THAT(c1.center.y, Catch::Matchers::WithinAbs(2.807354, 10e-5));
        REQUIRE_THAT(c1.radius, Catch::Matchers::WithinAbs(4.8275, 10e-5));
        REQUIRE_THAT(c2.center.x, Catch::Matchers::WithinAbs(0.4908357, 10e-5));
        REQUIRE_THAT(c2.center.y, Catch::Matchers::WithinAbs(-22.15212, 10e-5));
        REQUIRE_THAT(c2.radius, Catch::Matchers::WithinAbs(22.17979, 10e-5));
}