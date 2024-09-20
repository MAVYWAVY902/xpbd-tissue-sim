#ifndef __TEXT_RENDERING_VIEWER_HPP
#define __TEXT_RENDERING_VIEWER_HPP

#include <easy3d/viewer/viewer.h>
#include <easy3d/renderer/text_renderer.h>
#include <easy3d/core/types.h>
#include <easy3d/util/resource.h>

#include "Simulation.hpp"

class Simulation;

/** A class that extends easy3d::Viewer in order to render text in the graphics window.
 * Necessary so that the drawing of text can happen in the Viewer redraw, so that the text actually gets rendered.
 * TextRenderingViewer has support for as much rendered text as needed through specifications of TextSpecs, which
 * contain all information needed to render text on the screen.
 * 
 */
class TextRenderingViewer : public easy3d::Viewer
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
        easy3d::TextRenderer::Align alignment;
        Font font;
        easy3d::vec3 color;
        float line_spacing;
        bool upper_left;

        /** Constructor defined for convenience */
        TextSpec(const std::string& name_, 
                const std::string& text_,
                const float& x_,
                const float& y_,
                const float& font_size_,
                const easy3d::TextRenderer::Align& alignment_,
                const Font& font_,
                const easy3d::vec3& color_,
                const float& line_spacing_,
                const bool& upper_left_
                ) 
                : name(name_), text(text_), x(x_), y(y_), font_size(font_size_), alignment(alignment_),
                font(font_), color(color_), line_spacing(line_spacing_), upper_left(upper_left_)
        {

        }
    };

    /** Constructor - initialize the viewer with a title */
    explicit TextRenderingViewer(const std::string& title);

    /** Add text to the Viewer to be rendered
     * Simply creates a TextSpec according to the parameters passed in
     * Default parameters are used for convenience - at minimum, the name of the text and the text itself are required
     */
    void addText(const std::string& name,
                 const std::string& text,
                 const float& x = 0.0f,
                 const float& y = 0.0f,
                 const float& font_size = 20.0f,
                 const easy3d::TextRenderer::Align& alignment = easy3d::TextRenderer::ALIGN_LEFT,
                 const Font& font = Font::MAO,
                 const easy3d::vec3& color = easy3d::vec3(0.0f, 0.0f, 0.0f),
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

    /** Assigns a weak reference to the simulation.
     * @param simulation : a non-owning pointer to the Simulation object
     */
    void registerSimulation(Simulation* simulation) { _simulation = simulation; }

    protected:
    /** Overridden draw method from easy3d::Viewer, with added functionality to draw each TextSpec. */
    void draw() const override;
    
    /** Overridden from easy3d::Viewer, with added functionality to add each font to the TextRenderer. */
    void init() override;

    /** Draws each TextSpec. */
    void drawText() const;

    /** Triggered on key-press events */
    bool callback_event_keyboard(int key, int action, int modifiers) override;

    /** Triggered on mouse press events */
    bool callback_event_mouse_button(int button, int action, int modifiers) override;

    /** Triggered when mouse moves */
    bool callback_event_cursor_pos(double x, double y) override;

    private:
    /** The TextRenderer responsible for drawing the text on screen. */
    std::unique_ptr<easy3d::TextRenderer> _text_renderer;
    /** Stores each TextSpec under its name */
    std::map<std::string, TextSpec> _text_map;
    /** Non-owning pointer to the Simulation object that owns this Viewer
     * Used to notify Simulation object when a key is pressed (which applies when using the Frame-by-frame simulation mode)
     */
    Simulation* _simulation;
};

#endif // __TEXT_RENDERING_VIEWER_HPP