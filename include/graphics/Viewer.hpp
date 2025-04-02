#ifndef __VIEWER_HPP
#define __VIEWER_HPP

#include <string>
#include <map>
#include <array>

namespace Sim
{
    class Simulation;
}
namespace Graphics
{

class Viewer
{
    public:
     /** Font enum
     * Lists out the available fonts, in order that they are added as options to the TextRenderer
     */
    enum Font
    {
        MAO = 0,
        COUSINE = 1,
        EARTH = 2,
        GUNIT = 3,
        ROBOTO_BOLD = 4,
        ROBOTO_REGULAR = 5,
        VERA = 6
    };

    /** Text alignment enum
     * Lists out available text alignments for drawing text in the viewport.
     */
    enum TextAlignment
    {
        LEFT = 0,
        RIGHT,
        CENTER
    };

    /** Specification for drawn text
     * All of the below parameters are used in the TextRenderer::draw function
     * @param name : the name that this text can be searched by
     * @param text : the text to be rendered
     * @param x : the x position (before dpi scaling)
     * @param y : the y position (before dpi scaling)
     * @param font_size : the size of the font to be rendered
     * @param alignment : the alignment of the text (left, center or right)
     * @param font : the font to use - one of the options from the enum
     * @param color : the color of the text
     * @param line_spacing : the spacing between the lines of text
     * @param upper_left : when true, coordinates are relative to the upper left, otherwise w.r.t bottom left
     */
    struct TextSpec
    {
        std::string name;
        std::string text;
        float x;
        float y;
        float font_size;
        TextAlignment alignment;
        Font font;
        std::array<float,3> color;
        float line_spacing;
        bool upper_left;

        /** Constructor defined for convenience */
        TextSpec(const std::string& name_, 
                const std::string& text_,
                const float& x_,
                const float& y_,
                const float& font_size_,
                const TextAlignment alignment_,
                const Font& font_,
                const std::array<float,3>& color_,
                const float& line_spacing_,
                const bool& upper_left_
                ) 
                : name(name_), text(text_), x(x_), y(y_), font_size(font_size_), alignment(alignment_),
                font(font_), color(color_), line_spacing(line_spacing_), upper_left(upper_left_)
        {

        }
    };

    /** Constructor - initialize the viewer with a title */
    explicit Viewer(const std::string& name);

    /** Assigns a weak reference to the simulation.
     * @param simulation : a non-owning pointer to the Simulation object
     */
    void registerSimulation(Sim::Simulation* simulation) { _simulation = simulation; }

    /** Add text to the Viewer to be rendered
     * Simply creates a TextSpec according to the parameters passed in
     * Default parameters are used for convenience - at minimum, the name of the text and the text itself are required
     */
    void addText(const std::string& name,
                 const std::string& text,
                 const float& x = 0.0f,
                 const float& y = 0.0f,
                 const float& font_size = 20.0f,
                 const TextAlignment& alignment = TextAlignment::LEFT,
                 const Font& font = Font::MAO,
                 const std::array<float,3>& color = {0.0f, 0.0f, 0.0f},
                 const float& line_spacing = 0.5f,
                 const bool& upper_left = true);

    /** Removes a text with the specified name
     * @param name : the name of the TextSpec to remove
     */
    void removeText(const std::string& name);

    /** Modifies the text of a rendered TextSpec 
     * @param name : the name of the TextSpec to edit
     * @param new_text : the new text that the TextSpec should have
    */
    void editText(const std::string& name, const std::string& new_text);

    /** Modifies the text, position, and font size of a rendered TextSpec 
     * @param name : the name of the TextSpec to edit
     * @param new_text : the new text that the TextSpec should have
     * @param new_x : the new x position of the text
     * @param new_y : the new y position of the text
     * @param new_font_size : the new font size of the text
    */
    void editText(const std::string& name,
                  const std::string& new_text,
                  const float& new_x,
                  const float& new_y,
                  const float& new_font_size);

    /** Set whether or not to enable mouse interaction with the viewer.
     * Note: this will still pass mouse events along to the Simulation, but will disable the default response to mouse events by the Easy3d viewer.
     * @param allow : whether or not to enable mouse interaction with the viewer
     */
    void enableMouseInteraction(bool enable) { _enable_mouse_interaction = enable; }

    /** Updates internal graphics buffers and redraws.
     * NOTE: this is basically just a wrapper around the easy3d API and may change if more graphics backends are added
     */
    virtual void update() = 0;

    protected:

    // NOTE: the key, action, modifiers, arguments are from the Easy3D keyboard and mouse events. Subject to change as more graphics backends are added.

    /** Shared viewer behavior on keyboard events. */
    void _processKeyboardEvent(int key, int action, int modifiers);

    /** Shared viewer behavior on mouse button events. */
    void _processMouseButtonEvent(int button, int action, int modifiers);

    /** Shared viewer behavior on mouse move events. */
    void _processCursorMoveEvent(double x, double y);

    /** Shared viewer behavior on mouse scroll events. */
    void _processScrollEvent(double dx, double dy);

    std::string _name;

    /** Stores each TextSpec under its name */
    std::map<std::string, TextSpec> _text_map;

    /** Non-owning pointer to the Simulation object that owns this Viewer
     * Used to notify Simulation object when a key is pressed (which applies when using the Frame-by-frame simulation mode)
     */
    Sim::Simulation* _simulation;

    /** Whether or not the default Viewer response to mouse events is enabled.
     * E.g. normally when the user clicks and drags the Viewer window, the view of the camera is rotated. In some interactive sims we may not want that.
     */
    bool _enable_mouse_interaction;
};

} // namespace Graphics

#endif // __VIEWER_HPP