"""
Base class for radar displays providing common functionality
"""
from abc import ABC, abstractmethod
from PyQt6.QtCore import Qt, QRectF, QPointF
from PyQt6.QtGui import QPainter, QColor, QPen
from typing import Dict, Optional
import traceback
from FMOFP.Utils.logger.sys_logger import get_logger
from ..visual.theme_manager import get_theme_manager, DisplayTheme
from ..visual.enhanced_theme_manager import get_enhanced_theme_manager
from ..visual.effects import VisualEffects

logger = get_logger()

class BaseRadarDisplay(ABC):
    def __init__(self):
        self.range_scale = 40  # nautical miles

        # Size of the surface last painted; see viewport_rect() below.
        self._last_viewport_rect = QRectF()
        
        # Theme manager and visual effects
        self._theme_manager = get_theme_manager()
        self._enchanced_theme_manager = get_enhanced_theme_manager()
        self._visual_effects = VisualEffects()
        
        # Colors - will be updated from theme
        self.hud_color = QColor(0, 255, 0)  # Default HUD green
        self.warning_color = QColor(255, 255, 0)  # Yellow for warnings
        
        # Update colors from theme
        self.update_colors_from_theme()
    
    # ------------------------------------------------------------------ #
    # Viewport size
    #
    # These classes are plain objects owned by a QWidget (see
    # weather_radar_widget.py), not QWidget subclasses -- BaseRadarDisplay
    # derives from ABC. A good deal of the drawing code was nevertheless
    # written as though `self` were the widget, calling self.width() and
    # self.height(). Those calls raised AttributeError every time, and because
    # each drawing method wraps its body in `except Exception: log`, the failure
    # showed up only as a log line: the weather radar's entire particle render
    # path ("Error drawing particles: 'WeatherRadarDisplay' object has no
    # attribute 'width'") and every draw method in the holographic weather radar
    # silently produced nothing at all.
    #
    # The size is taken from the QPainter's own device rather than cached on the
    # instance, so it is always the surface actually being painted and there is
    # no stale value to go wrong between resizes. Pass the painter you are
    # drawing with.
    # ------------------------------------------------------------------ #

    def viewport_rect(self, painter: QPainter) -> QRectF:
        """Return the rectangle of the surface `painter` is drawing on.

        The result is also remembered, so code that runs outside a paint cycle
        (positioning a panel in response to a click, say) can ask
        last_viewport_rect() for the size of the surface last painted.
        """
        try:
            viewport = painter.viewport()
            if viewport is not None and viewport.width() > 0 and viewport.height() > 0:
                self._last_viewport_rect = QRectF(viewport)
                return QRectF(self._last_viewport_rect)
        except Exception:
            pass

        try:
            device = painter.device()
            if device is not None and device.width() > 0 and device.height() > 0:
                self._last_viewport_rect = QRectF(
                    0.0, 0.0, float(device.width()), float(device.height()))
                return QRectF(self._last_viewport_rect)
        except Exception:
            pass

        # Neither the viewport nor the device could be read. Returning a null
        # rect is honest: callers use it for culling and centring, and a made-up
        # size would put echoes somewhere they are not.
        logger.warning(
            "[BASE_RADAR] Could not determine the paint surface size from the "
            "painter; returning a null viewport"
        )
        return QRectF()

    def viewport_width(self, painter: QPainter) -> float:
        """Width of the surface `painter` is drawing on."""
        return self.viewport_rect(painter).width()

    def viewport_height(self, painter: QPainter) -> float:
        """Height of the surface `painter` is drawing on."""
        return self.viewport_rect(painter).height()

    def last_viewport_rect(self) -> QRectF:
        """The surface size recorded by the most recent viewport_rect() call.

        Null until something has been painted.  Callers with no painter to hand
        must treat a null rect as "not known yet" and do nothing that depends on
        the size, rather than substituting one.
        """
        return QRectF(getattr(self, '_last_viewport_rect', QRectF()))

    def update_colors_from_theme(self):
        """Update display colors from current theme"""
        self.hud_color = self._theme_manager.get_color("hud")
        self.warning_color = self._theme_manager.get_color("warning")

    def set_theme(self, theme: DisplayTheme) -> bool:
        """Set display theme and update"""
        if self._theme_manager.set_theme(theme):
            self.update_colors_from_theme()
            return True
        return False

    def draw_display(self, painter: QPainter, rect: QRectF, data: Dict):
        """Main draw method with error handling"""
        try:
            # Save state
            painter.save()
            
            # Enable antialiasing
            painter.setRenderHint(QPainter.RenderHint.Antialiasing)
            painter.setRenderHint(QPainter.RenderHint.TextAntialiasing)
            
            # Draw radar elements
            self.draw_radar_elements(painter, rect, data)
            
            # Restore state
            painter.restore()
            
        except Exception as e:
            logger.error(f"Error drawing radar display: {str(e)}")
            logger.error(traceback.format_exc())
            self._draw_error_message(painter, rect, str(e))

    @abstractmethod
    def draw_radar_elements(self, painter: QPainter, rect: QRectF, data: Dict):
        """Draw radar-specific elements"""
        pass

    def draw_range_rings(self, painter: QPainter, center: QPointF, radius: float):
        """Draw standard range rings with enhanced visuals"""
        try:
            # Get styling parameters from theme
            use_gradients = self._theme_manager.get_style_param("use_gradients", False)
            line_width = self._theme_manager.get_style_param("line_width", 1.0)
            
            # Draw range rings with enhanced styling
            for r in [radius/3, radius*2/3, radius]:
                ring_rect = QRectF(
                    center.x() - r,
                    center.y() - r,
                    r * 2,
                    r * 2
                )
                
                if use_gradients:
                    # Draw with enhanced effects
                    pen = QPen(self.hud_color)
                    pen.setWidthF(line_width)
                    
                    # Use dashed line for middle ring
                    if r == radius*2/3:
                        pen.setStyle(Qt.PenStyle.DashLine)
                    
                    painter.setPen(pen)
                    
                    # Draw with slight glow for outer ring
                    if r == radius:
                        self._visual_effects.draw_enhanced_ellipse(
                            painter, ring_rect, 
                            color=self.hud_color,
                            glow=True
                        )
                    else:
                        painter.drawEllipse(ring_rect)
                else:
                    # Fall back to basic drawing
                    painter.setPen(self.hud_color)
                    painter.drawEllipse(ring_rect)
                
                # Draw range labels with enhanced text
                range_nm = int(self.range_scale * (r/radius))
                label_rect = QRectF(
                    center.x() - 20,  # x
                    center.y() - r - 20,  # y
                    40,  # width
                    15   # height
                )
                
                if use_gradients:
                    self._visual_effects.draw_enhanced_text(
                        painter, label_rect, 
                        Qt.AlignmentFlag.AlignCenter,
                        f"{range_nm}nm",
                        glow=r == radius,  # Only glow the outer ring label
                        glow_color=self.hud_color
                    )
                else:
                    painter.drawText(
                        label_rect,
                        Qt.AlignmentFlag.AlignCenter,
                        f"{range_nm}nm"
                    )
        except Exception as e:
            logger.error(f"Error drawing range rings: {str(e)}")
            raise

    def _draw_error_message(self, painter: QPainter, rect: QRectF, message: str):
        """Draw error message on display with enhanced visuals"""
        try:
            use_gradients = self._theme_manager.get_style_param("use_gradients", False)
            
            if use_gradients:
                error_color = QColor(255, 30, 30, 220)  # Bright red
                self._visual_effects.draw_enhanced_text(
                    painter, rect, 
                    Qt.AlignmentFlag.AlignCenter,
                    f"Error: {message}",
                    glow=True,
                    glow_color=error_color
                )
            else:
                painter.setPen(QColor(255, 0, 0))  # Red for errors
                painter.drawText(rect, Qt.AlignmentFlag.AlignCenter, f"Error: {message}")
        except Exception as e:
            logger.error(f"Error drawing error message: {str(e)}")

    def draw_text(self, painter: QPainter, rect: QRectF, flags: int, text: str, 
                 color: Optional[QColor] = None, glow: bool = False):
        """Draw text with theme-aware enhancements"""
        if color is None:
            color = self.hud_color
        
        use_effects = self._theme_manager.get_style_param("use_gradients", False)
        if use_effects:
            self._visual_effects.draw_enhanced_text(
                painter, rect, flags, text, 
                glow=glow, 
                glow_color=color,
                shadow=self._theme_manager.get_style_param("use_shadows", False)
            )
        else:
            # Fall back to basic drawing if effects disabled
            painter.setPen(color)
            painter.drawText(rect, flags, text)
    
    def draw_line(self, painter: QPainter, start: QPointF, end: QPointF, 
                 color: Optional[QColor] = None, width: float = 1.0, glow: bool = False):
        """Draw line with theme-aware enhancements"""
        if color is None:
            color = self.hud_color
        
        use_effects = self._theme_manager.get_style_param("use_gradients", False)
        if use_effects:
            self._visual_effects.draw_enhanced_line(
                painter, start, end, 
                color=color,
                width=width,
                glow=glow
            )
        else:
            # Fall back to basic drawing if effects disabled
            pen = QPen(color)
            pen.setWidthF(width)
            painter.setPen(pen)
            painter.drawLine(start, end)

    def world_to_screen(self, world_pos: tuple, center: QPointF, 
                       radius: float, range_scale: float) -> QPointF:
        """Convert world coordinates to screen coordinates"""
        try:
            x = center.x() + (world_pos[0] / range_scale) * radius
            y = center.y() - (world_pos[1] / range_scale) * radius
            return QPointF(x, y)
        except Exception as e:
            logger.error(f"Error converting coordinates: {str(e)}")
            return center  # Return center as fallback
