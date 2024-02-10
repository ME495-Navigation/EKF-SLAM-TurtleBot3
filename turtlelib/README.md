# Turtlelib Library
A library for handling transformations in SE(2) and other turtlebot-related math.

# Components
- geometry2d - Handles 2D geometry primitives
- se2d - Handles 2D rigid body transformations
- svg - Handles SVG visualization
- diff_drive - Handles differential drive kinematics
- frame_main - Perform some rigid body computations based on user input

# Conceptual Questions
1. If you needed to be able to `normalize` Vector2D objects (i.e., find the unit vector in the direction of a given Vector2D):
   - Propose three different designs for implementing the ~normalize~ functionality
   
   - Discuss the pros and cons of each proposed method, in light of the C++ Core Guidelines.

   - Which of the methods would you implement and why?
    
    Goals for the `normalize` functionality -
    1. The functionality should be associated with Vector2D objects (C++ Core Guidelines C.5).
    2. It shouldn't be a function member since it does not need direct access to the representation of a class (C++ Core Guidelines C.4).
    3. Operations within the `normalize` functionality must themselves be standalone functions (C++ Core Guidelines F.10)

    Design 1: The normalize function is a member function of the Vector2D struct.
    - Pros - Direct association with the Vector2D object.
    - Cons - The function can be interpreted as a method to change state of the object since Vector2D is a struct, which is not desirable.

    Design 2: The normalize function is a separate function outside the Vector2D struct. The function returns a Vector2D object..
    - Pros - The function is standalone and does not modify the original object.
    - Cons - All the functionality is implemented in a single function. Operations within cannot be reused independently. 

    Design 3: The `normalize`` functionality is implemented as a standalone function. The operations within are defined as separate functions.
    - Pros - Greater opportunity for reuse and increased readablity.
    - Cons - Reduced association with the Vector2D objects.

    After carefully considering the design choices described above, I chose to implement the `normalize` functionality according to Design 3. Implementing the
    magnitude operation as a separate function increases reusabilty and since the `normalize` function is implemented inside the same namespace, association is
    maintained.

2. What is the difference between a class and a struct in C++?

    - The members within a class are private by default and the members within a struct are public by default in C++. The two types are identical otherwise.


3. Why is Vector2D a struct and Transform2D a Class (refer to at least 2 specific C++ core guidelines in your answer)?

    - Vector2D is a struct because the data members inside can vary independently and are allowed to be modfied (variant) (C++ Core Guidelines C.2).
    - Transform2D is a class since it contains non-public members which should not be modified (invariant) (C++ Core Guidelines C.2, C.8). 


4. Why are some of the constructors in Transform2D explicit (refer to a specific C++ core guideline in your answer)?

    - The Transform2D constructors are single argument and overloaded. Implicit type conversion may result in a call to a different constructor.
      C++ Core Guidelines C.46 - By default, declare single-argument constructors explicit. For this reason the contructors in Transform2D are explicit.


5. Why is Transform2D::inv() declared const while Transform2D::operator*=() is not?
   - Refer to [[https://isocpp.github.io/CppCoreGuidelines/CppCoreGuidelines#con-constants-and-immutability][C++ Core Guidelines (Constants and Immutability)]] in your answer

   - The guidelines (Con.2) state that a member function should be marked const unless it changes the object’s observable state. This is why Transform2D::inv() is declared const. On the other hand, since T.ransform2D::operator*=() changes modifies the input (changes the object's observable state), it is not declared const.