#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <iostream>
#include <iosfwd> // contains forward definitions for iostream objects
#include <cmath>
#include <sstream>
#include <turtlelib/svg.hpp>
#include <turtlelib/se2d.hpp>
#include <turtlelib/geometry2d.hpp>

namespace turtlelib{
TEST_CASE( "svg output", "[svg]" )
{
    Svg svg("/tmp/frames.svg");

    Transform2D A,t_ab, t_bc;
    Point2D p_a;
    Vector2D v_b;

    SvgPoint p_1,p_2,p_3;
    SvgLine l_a,l_b,l_c;
    
    std::stringstream is_1,is_2,is_3,is_4;
    is_1 << "0 0 1";
    is_2 << "-90 0 -2";
    is_3 << "1.5 1.5";
    is_4 << "1 1";
    
    is_1 >> t_ab;
    is_2 >> t_bc;
    is_3 >> p_a;
    is_4 >> v_b;

    svg.draw_frame(A,"{A}");
    svg.draw_frame(t_ab,"{B}");
    svg.draw_frame(t_ab*t_bc,"{C}");

    p_1.point = p_a;
    p_2.point = (t_ab)((t_ab.inv())(p_a));
    p_2.fill = "brown";
    p_3.point = (t_ab*t_bc)(((t_ab*t_bc).inv())(p_a));
    p_3.fill = "orange";
    svg.draw_point(p_1);
    svg.draw_point(p_2);
    svg.draw_point(p_3);

    l_b.x2 = t_ab.translation().x;
    l_b.y2 = t_ab.translation().y;
    l_b.x1 = t_ab(normalize(v_b)).x + l_b.x2;
    l_b.y1 = t_ab(normalize(v_b)).y + l_b.y2;
    l_b.stroke = "brown";
    svg.draw_line(l_b);

    l_a.x2 = 0.0;
    l_a.y2 = 0.0;
    l_a.x1 = t_ab(v_b).x + l_a.x2 ;
    l_a.y1 = t_ab(v_b).y + l_a.y2;
    l_a.stroke = "purple";
    svg.draw_line(l_a);

    l_c.x2 = (t_ab*t_bc).translation().x;
    l_c.y2 = (t_ab*t_bc).translation().y;
    l_c.x1 = (t_ab(v_b)).x + (t_ab*t_bc).translation().x; 
    l_c.y1 = (t_ab(v_b)).y + (t_ab*t_bc).translation().y;
    l_c.stroke = "orange";
    svg.draw_line(l_c);

    REQUIRE( SvgOutput("/tmp/frames.svg") == 
R"delimiter(<svg width="8.500000in" height="11.000000in" viewBox="0 0 816.000000 1056.000000" xmlns="http://www.w3.org/2000/svg">

<defs>
<marker
    style="overflow:visible"
    id="Arrow1Sstart"
    refX="0.0"
    refY="0.0"
    orient="auto">
    <path
        transform="scale(0.2) translate(6,0)"
        style="fill-rule:evenodd;fill:context-stroke;stroke:context-stroke;stroke-width:1.0pt"
        d="M 0.0,0.0 L 5.0,-5.0 L -12.5,0.0 L 5.0,5.0 L 0.0,0.0 z "
        />
    </marker>
</defs>

<g>
<line x1="504" x2="408" y1="528" y2="528" stroke="red" stroke-width="5" marker-start="url(#Arrow1Sstart)" /> />
<line x1="408" x2="408" y1="432" y2="528" stroke="green" stroke-width="5" marker-start="url(#Arrow1Sstart)" /> />
<text x="417.6" y="518.4">{A}</text>
</g>
<g>
<line x1="504" x2="408" y1="432" y2="432" stroke="red" stroke-width="5" marker-start="url(#Arrow1Sstart)" /> />
<line x1="408" x2="408" y1="336" y2="432" stroke="green" stroke-width="5" marker-start="url(#Arrow1Sstart)" /> />
<text x="417.6" y="422.4">{B}</text>
</g>
<g>
<line x1="408" x2="408" y1="720" y2="624" stroke="red" stroke-width="5" marker-start="url(#Arrow1Sstart)" /> />
<line x1="504" x2="408" y1="624" y2="624" stroke="green" stroke-width="5" marker-start="url(#Arrow1Sstart)" /> />
<text x="417.6" y="614.4">{C}</text>
</g>
<circle cx="552" cy="384" r="3" stroke="purple" fill="purple" stroke-width="1" />
<circle cx="552" cy="384" r="3" stroke="brown" fill="brown" stroke-width="1" />
<circle cx="552" cy="384" r="3" stroke="orange" fill="orange" stroke-width="1" />
<line x1="475.882" x2="408" y1="364.118" y2="432" stroke="brown" stroke-width="5" marker-start="url(#Arrow1Sstart)" /> />
<line x1="504" x2="408" y1="432" y2="528" stroke="purple" stroke-width="5" marker-start="url(#Arrow1Sstart)" /> />
<line x1="504" x2="408" y1="528" y2="624" stroke="orange" stroke-width="5" marker-start="url(#Arrow1Sstart)" /> />
)delimiter");
}

}