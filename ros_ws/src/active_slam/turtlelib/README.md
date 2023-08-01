# Turtlelib Library
A library for handling transformations in SE(2) and other turtlebot-related math.

# Components
- rigid2d - Handles 2D rigid body transformations
- frame_main - Perform some rigid body computations based on user input
- diff_drive - Computes inverse and forward kinematics for a robot

# Conceptual Questions
1. We need to be able to ~normalize~ Vector2D objects (i.e., find the unit vector in the direction of a given Vector2D):
   - Propose three different designs for implementing the ~normalize~ functionality

        One way to implement could be to split up the steps of normalizing the vector into different general functions, so that they could be used for anything. For instance, I could make a function that computes the magnitude of a vector and then one that divides each component by the magnitude. Then I could make a function that combines these steps to normalize the vector. Another method could be to add the function straight into the struct, since it only uses the variables of the struct. Finally, a third approach to implement this would be to make a template member function to normalize the vector, which could allow you to normalize a vector with its x and y components of any type.

   - Discuss the pros and cons of each proposed method, in light of the C++ Core Guidelines.

      Based on the C++ Core Guideline F.2, the first proposed idea would be good because the individual functions should only perform a single logical operation, and so then these functions can be used for other operations besides just normalizing a vector. However, while smaller functions are easier to read and therefore preferred by the F.3 guideline, a con is that you have to make a lot more functions. For the second approach, based on the guideline F.15, it is a simple way of passing information and can directly reference the public variables. And, based on the F.15 guideline, A function that only passes an out variable is cheaper in terms of overhead than one passing an in and out. However, a con is that it is a specific function solely for normalizing a vector. For the final approach, a benefit would be that if you are trying to normalize a vector with small values, then you can use x and y variables of type float, or even change to int, to decrease the overhead of the output, according to guideline T.2. A con of this approach, however, is that the code can be less efficient because the compiler will generate a version of the function for each variable type used, therefore if you only need the function for one specific variable type, it's better to not use a template.

   - Which of the methods would you implement and why?

      I would implement the first approach because I think it's best to have individual functions to only perform one operation so that they can constantly be reused instead of making a bunch of very specific functions that contain similar steps. Plus, since smaller functions are easier to read, then it would be easier for someone else to understand your code, especially when working with other people.

2. What is the difference between a class and a struct in C++?

    By default, everything in a struct is public, whereas a class can be made up of private and public members.

3. Why is Vector2D a struct and Transform2D a Class (refer to at least 2 specific C++ core guidelines in your answer)?

   Based on the C.2 core guideline, Vector2D is a struct because the data members can vary independently, whereas the Transform 2D class has an invariant so it is a class. Additionally, based on the C.9 core guideline, Transform2D is a class because we wanted to enforce a relation among members, so we made the variables private so that we can minimize exposure so that only specific functions can change the members of the class. However, for Vector2D, it's okay for anyone to access and change those variables, so it is okay to just be a struct with all of it's members public and easy to access.

4. Why are some of the constructors in Transform2D explicit (refer to a specific C++ core guideline in your answer)?

   According to the C.46 core guideline, some of the constructors in Transform2D are explicit so that unintended conversions are avoided.

5. Why is Transform2D::inv() declared const while Transform2D::operator*=() is not?
   - Refer to [[https://isocpp.github.io/CppCoreGuidelines/CppCoreGuidelines#con-constants-and-immutability][C++ Core Guidelines (Constants and Immutability)]] in your answer

   Transform2D::inv() is declared const because then, according to Con.2 guideline, you can't change the state of the original variable. When using Transform2D::inv(), we only want to create a new inverted transform to return while not changing the original. If didn't declare the Transform2D::inv() as const, then the inverted transform would affect the rest of the functions. On the other hand, Transform2D::operator*=() is not const because we want to change the state of the original variable to be the variable times another transformation. However, we do not want to change the input transformation, so that input transformation that the transform is being multiplied by is constant.

Worked with Liz, Marno, Oubre, and Dilan