#ifndef __TEXT_RENDERING_VIEWER_HPP
#define __TEXT_RENDERING_VIEWER_HPP

#include <easy3d/viewer/viewer.h>
#include <easy3d/renderer/text_renderer.h>
#include <easy3d/core/types.h>
#include <easy3d/util/resource.h>

#include "graphics/Viewer.hpp"
#include "simulation/Simulation.hpp"

class Simulation;

namespace Graphics
{

/** A class that extends easy3d::Viewer in order to render text in the graphics window.
 * Necessary so that the drawing of text can happen in the Viewer redraw, so that the text actually gets rendered.
 * TextRenderingViewer has support for as much rendered text as needed through specifications of TextSpecs, which
 * contain all information needed to render text on the screen.
 * 
 */
class Easy3DTextRenderingViewer : public easy3d::Viewer, public Viewer
{
    public:

    /** Constructor - initialize the viewer with a title */
    explicit Easy3DTextRenderingViewer(const std::string& title);


    /** Updates internal graphics buffers and redraws the viewport.
     * Just a wrapper around easy3d::Viewer::update(), but needed to conform to the Graphics::Viewer specification.
     */
    virtual void update() override;

    /** Width of the viewer window. */
    virtual int width() const override { return easy3d::Viewer::width(); }

    /** Height of the viewer window. */
    virtual int height() const override { return easy3d::Viewer::height(); }

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

    /** Triggered when mouse is scrolled */
    bool callback_event_scroll(double dx, double dy) override;


    private:
    easy3d::TextRenderer::Align _getEasy3dAlignment(const TextAlignment& alignment) const;

    int _getEasy3dFontIndex(const Font& font) const;

    easy3d::vec3 _getEasy3dColor(const std::array<float,3>& color) const;

    private:
    /** The TextRenderer responsible for drawing the text on screen. */
    std::unique_ptr<easy3d::TextRenderer> _text_renderer;
};

} // namespace Graphics

#endif // __TEXT_RENDERING_VIEWER_HPP