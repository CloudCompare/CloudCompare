#pragma once
//##########################################################################
//#                                                                        #
//#                              CLOUDCOMPARE                              #
//#                                                                        #
//#  This program is free software; you can redistribute it and/or modify  #
//#  it under the terms of the GNU General Public License as published by  #
//#  the Free Software Foundation; version 2 or later of the License.      #
//#                                                                        #
//#  This program is distributed in the hope that it will be useful,       #
//#  but WITHOUT ANY WARRANTY; without even the implied warranty of        #
//#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the          #
//#  GNU General Public License for more details.                          #
//#                                                                        #
//#          COPYRIGHT: EDF R&D / TELECOM ParisTech (ENST-TSI)             #
//#                                                                        #
//##########################################################################

//Inspired from ccColorScaleEditorWidget by Richard Steffen (LGPL 2.1)

#include "CCPluginAPI.h"

//Qt
#include <QWidget>

//qCC_db
#include <ccColorScale.h>
/**
 * \brief Enumeration of CloudCompare entity types
 * 
 * \details Defines the basic types of entities that can be processed in the command line interface
 * 
 * Possible entity types:
 * - GROUP: A collection of other entities
 * - CLOUD: A point cloud entity
 * - MESH: A mesh entity
 */
enum class CL_ENTITY_TYPE {
	GROUP,   ///< Represents a group of entities
	CLOUD,   ///< Represents a point cloud
	MESH     ///< Represents a mesh
};

/**
 * \brief Base descriptor for loaded entities in CloudCompare
 * 
 * \details Provides a common interface for describing and accessing different types of entities
 * 
 * Key features:
 * - Stores entity metadata like basename, path, and file index
 * - Abstract base class for specific entity type descriptors
 * - Supports polymorphic entity retrieval
 * 
 * \note Serves as a base class for group, cloud, and mesh descriptors
 * 
 * \example
 * \code
 * // Concrete implementation in derived classes like CLCloudDesc or CLMeshDesc
 * class CLCloudDesc : public CLEntityDesc {
 *     ccPointCloud* pc;
 *     
 *     ccHObject* getEntity() override {
 *         return pc;
 *     }
 *     
 *     CL_ENTITY_TYPE getCLEntityType() const override {
 *         return CL_ENTITY_TYPE::CLOUD;
 *     }
 * };
 * \endcode
 */
struct CCPLUGIN_LIB_API CLEntityDesc
{
	// ... existing implementation
};
//! Set of color scale elements (widgets)
/** \brief Manages a collection of ColorScaleElementSlider widgets.
    \details This class provides methods to add, sort, and manage a set of color scale element sliders.
    
    The ColorScaleElementSliders class is designed to handle a dynamic collection of ColorScaleElementSlider objects,
    providing convenient methods for manipulation and access. Key features include:
    - Adding sliders while maintaining sorted order
    - Removing specific sliders
    - Querying slider information
    - Managing slider selection
    
    \code{.cpp}
    // Example usage of ColorScaleElementSliders
    ColorScaleElementSliders sliders;
    
    // Create and add sliders
    ColorScaleElementSlider* slider1 = new ColorScaleElementSlider();
    ColorScaleElementSlider* slider2 = new ColorScaleElementSlider();
    
    sliders.addSlider(slider1);  // Automatically sorts the list
    sliders.addSlider(slider2);
    
    // Get information about sliders
    int totalSliders = sliders.size();
    ColorScaleElementSlider* selectedSlider = sliders.element(sliders.selected());
    
    // Remove a slider
    sliders.removeAt(0);
    
    // Clear all sliders
    sliders.clear();
    \endcode
*/
class CCPLUGIN_LIB_API ColorScaleElementSliders
{
public:

	//! Type of the set of elements
	using Set = QList<ColorScaleElementSlider*>;

	//! Adds a slider element and sort the whole set
	/** \brief Adds a new slider to the collection and automatically sorts the elements.
	 *  \param slider Pointer to the ColorScaleElementSlider to be added
	 *  \note This method should be used instead of direct push_back/push_front operations
	 *  \warning The method takes ownership of the slider pointer
	 */
	void addSlider(ColorScaleElementSlider* slider);

	//! Returns the size (shortcut)
	/** \brief Gets the number of sliders in the collection
	 *  \return Current number of sliders
	 */
	inline int size() const { return m_list.size(); }

	//! Sorts the set
	/** \brief Sorts the sliders based on their internal ordering criteria
	 *  \note This method is typically called automatically by addSlider()
	 */
	void sort();

	//! Remove all sliders
	/** \brief Clears the entire collection of sliders
	 *  \note This method will delete all slider objects
	 */
	void clear();

	//! Remove a given slider
	/** \brief Removes a slider at the specified index
	 *  \param i Index of the slider to remove
	 *  \note The slider at the specified index will be deleted
	 *  \throws std::out_of_range if index is invalid
	 */
	void removeAt(int i);

	//! Returns the currently selected slider index (or -1 if none)
	/** \brief Gets the index of the currently selected slider
	 *  \return Index of the selected slider, or -1 if no slider is selected
	 */
	int selected() const;

	//! Returns the index of a given slider
	/** \brief Finds the index of a specific slider in the collection
	 *  \param slider Pointer to the slider to find
	 *  \return Index of the slider, or -1 if not found
	 */
	int indexOf(ColorScaleElementSlider* slider);

	//! Returns a given element
	/** \brief Retrieves a slider at the specified index
	 *  \param index Index of the slider to retrieve
	 *  \return Pointer to the ColorScaleElementSlider at the given index
	 *  \throws std::out_of_range if index is invalid
	 */
	inline ColorScaleElementSlider* element(int index) { return m_list.at(index); }
	
	//! Returns a given element (const version)
	/** \brief Retrieves a slider at the specified index (const version)
	 *  \param index Index of the slider to retrieve
	 *  \return Const pointer to the ColorScaleElementSlider at the given index
	 *  \throws std::out_of_range if index is invalid
	 */
	inline const ColorScaleElementSlider* element(int index) const { return m_list.at(index); }

	//! Return the set of elements
	/** \brief Gets a reference to the internal collection of sliders
	 *  \return Reference to the set of ColorScaleElementSlider pointers
	 */
	inline Set& elements() { return m_list; }
	
	//! Return the set of elements (const version)
	/** \brief Gets a const reference to the internal collection of sliders
	 *  \return Const reference to the set of ColorScaleElementSlider pointers
	 */
	inline const Set& elements() const { return m_list; }

protected:

	//! Set of elements
	Set m_list;
};

//! Shared set of color scale elements (widgets)
/** \brief Smart pointer type for managing ColorScaleElementSliders
 *  \details Provides reference-counted shared ownership of ColorScaleElementSliders
 */
using SharedColorScaleElementSliders = QSharedPointer<ColorScaleElementSliders>;

//! Base color scale editor (sub)Widget
/** \brief A base widget for color scale editing.
    \details This widget provides a margin along a preferred orientation and manages associated sliders.
    
    The ColorScaleEditorBaseWidget serves as a base class for creating color scale editor widgets
    with flexible orientation and margin configuration. Key features include:
    - Configurable widget orientation (horizontal or vertical)
    - Adjustable margin for precise layout control
    - Shared management of color scale element sliders
    
    \code{.cpp}
    // Example of creating a color scale editor widget
    // Create a shared set of sliders
    SharedColorScaleElementSliders sliders = QSharedPointer<ColorScaleElementSliders>::create();
    
    // Create a horizontal color scale editor with 10-pixel margin
    ColorScaleEditorBaseWidget* editor = new ColorScaleEditorBaseWidget(
        sliders,               // Shared sliders
        Qt::Horizontal,        // Orientation
        10                     // Margin
    );
    
    // Modify sliders or orientation as needed
    editor->setSliders(sliders);
    Qt::Orientation currentOrientation = editor->getOrientation();
    int currentMargin = editor->getMargin();
    
    // Calculate useful length (widget width/height minus margins)
    int usableLength = editor->length();
    \endcode
    
    \note This is an abstract base class intended to be subclassed for specific color scale editing implementations.
    \see ColorScaleElementSliders
*/
class CCPLUGIN_LIB_API ColorScaleEditorBaseWidget : public QWidget
{
	Q_OBJECT

public:

	//! Default constructor
	/** \brief Constructs a color scale editor widget
	 *  \param sliders Shared pointer to the set of color scale element sliders
	 *  \param orientation Widget orientation (horizontal or vertical)
	 *  \param margin Margin size in pixels to provide padding around the widget
	 *  \param parent Optional parent widget for ownership and layout
	 *  
	 *  \details Initializes the widget with specified sliders, orientation, and margin.
	 *  The margin provides space around the edges of the widget, which can be useful 
	 *  for creating visual separation or accommodating border effects.
	 *  
	 *  \note The orientation determines how the color scale will be displayed:
	 *  - Qt::Horizontal: Color scale runs from left to right
	 *  - Qt::Vertical: Color scale runs from top to bottom
	 */
	ColorScaleEditorBaseWidget(SharedColorScaleElementSliders sliders, 
		Qt::Orientation orientation,
		int margin,
		QWidget* parent = nullptr)
		: QWidget(parent)
		, m_sliders(sliders)
		, m_orientation(orientation)
		, m_margin(margin)
	{}

	//! Returns useful length
	/** \brief Calculates the usable length of the widget
	 *  \return Available widget length minus margins
	 *  \details Computes the widget's width or height based on orientation, 
	 *  subtracting the specified margin from both sides.
	 *  
	 *  For horizontal orientation, returns width - (2 * margin)
	 *  For vertical orientation, returns height - (2 * margin)
	 */
	int length() const { return (m_orientation == Qt::Horizontal ? contentsRect().width() : contentsRect().height())-2*m_margin; }

	//! Sets associated sliders set
	/** \brief Updates the set of color scale sliders for the widget
	 *  \param sliders New shared pointer to ColorScaleElementSliders
	 *  \details Replaces the current set of sliders and triggers a widget update
	 *  
	 *  \note This is a virtual method that can be overridden by derived classes
	 *  to provide additional update logic
	 */
	virtual void setSliders(SharedColorScaleElementSliders sliders) { m_sliders = sliders; update(); }

	//! Returns orientation
	/** \brief Gets the current widget orientation
	 *  \return Current orientation (horizontal or vertical)
	 */
	Qt::Orientation getOrientation() const { return m_orientation; }
	
	//! Returns margin
	/** \brief Gets the current widget margin
	 *  \return Margin size in pixels
	 */
	int getMargin() const { return m_margin; }

protected:

	//! Associated sliders
	SharedColorScaleElementSliders m_sliders;

	//! Orientation
	Qt::Orientation m_orientation;

	//! Margin
	int m_margin;
};

//! Color bar widget
/** \brief A widget representing a color bar for color scales.
    \details This class inherits from ColorScaleEditorBaseWidget and handles mouse events for color selection.
    
    The ColorBarWidget provides a visual representation of a color scale with interactive capabilities.
    Key features include:
    - Displays a color gradient based on associated color scale sliders
    - Supports horizontal and vertical orientations
    - Emits a signal when a point on the color bar is clicked
    
    \code{.cpp}
    // Example of creating and using a ColorBarWidget
    // Create a shared set of color scale sliders
    SharedColorScaleElementSliders sliders = QSharedPointer<ColorScaleElementSliders>::create();
    
    // Add some color scale elements to the sliders
    ColorScaleElementSlider* slider1 = new ColorScaleElementSlider();
    slider1->setPosition(0.0);  // Start of the color scale
    slider1->setColor(Qt::blue);
    
    ColorScaleElementSlider* slider2 = new ColorScaleElementSlider();
    slider2->setPosition(1.0);  // End of the color scale
    slider2->setColor(Qt::red);
    
    sliders->addSlider(slider1);
    sliders->addSlider(slider2);
    
    // Create a horizontal color bar widget
    ColorBarWidget* colorBar = new ColorBarWidget(sliders);
    
    // Connect to the point clicked signal
    QObject::connect(colorBar, &ColorBarWidget::pointClicked, 
        [](double relativePos) {
            // Handle color selection
            qDebug() << "Color selected at position: " << relativePos;
        });
    \endcode
    
    \note The widget supports both horizontal and vertical color bar orientations.
    \see ColorScaleEditorBaseWidget, ColorScaleElementSliders
*/
class CCPLUGIN_LIB_API ColorBarWidget : public ColorScaleEditorBaseWidget
{
	Q_OBJECT

public:

	//! Default constructor
	/** \brief Constructs a color bar widget
	 *  \param sliders Shared pointer to the set of color scale element sliders
	 *  \param parent Optional parent widget for ownership and layout
	 *  \param orientation Widget orientation (horizontal or vertical), defaults to horizontal
	 *  
	 *  \details Initializes the color bar widget with the given sliders and orientation.
	 *  The widget will use the provided color scale sliders to render the color gradient.
	 *  
	 *  \note If no orientation is specified, the color bar will be horizontal by default.
	 *  
	 *  \see setSliders() to update the color scale sliders after construction
	 */
	ColorBarWidget(SharedColorScaleElementSliders sliders, QWidget* parent = nullptr, Qt::Orientation orientation = Qt::Horizontal);

Q_SIGNALS:

	//! Signal emitted when the mouse (left) button is clicked
	/** \brief Notifies listeners about a color selection event
	 *  \param relativePos Relative click position along the color bar
	 *  
	 *  \details This signal is emitted when a user clicks on the color bar.
	 *  The relative position is a normalized value between 0 and 1:
	 *  - 0.0 represents the start of the color bar
	 *  - 1.0 represents the end of the color bar
	 *  
	 *  \note The signal can be used to implement interactive color selection 
	 *  or to track user interactions with the color bar.
	 *  
	 *  \par Example:
	 *  \code{.cpp}
	 *  // Connecting to the pointClicked signal
	 *  ColorBarWidget* colorBar = new ColorBarWidget(sliders);
	 *  connect(colorBar, &ColorBarWidget::pointClicked, 
	 *      [](double relativePos) {
	 *          // Retrieve color at the clicked position
	 *          QColor selectedColor = colorBar->getColorAtPosition(relativePos);
	 *      });
	 *  \endcode
	 */
	void pointClicked(double relativePos);

protected:

	//inherited from QWidget
	/** \brief Handles the paint event for rendering the color bar
	 *  \param e Pointer to the paint event
	 *  
	 *  \details Overrides the default QWidget paint event to draw 
	 *  the color gradient based on associated color scale sliders.
	 *  
	 *  \note This method is called automatically by Qt's event system 
	 *  when the widget needs to be redrawn.
	 */
	void paintEvent(QPaintEvent* e) override;
	/** \brief Handles mouse press events for color selection
	 *  \param e Pointer to the mouse press event
	 *  
	 *  \details Captures mouse press events to determine the relative 
	 *  position of the click along the color bar.
	 *  
	 *  \note Emits the pointClicked signal with the relative position.
	 */
	void mousePressEvent(QMouseEvent* e) override;
};

//! All sliders widget
/** \brief A widget that manages multiple color scale sliders.
    \details This class allows for the addition, selection, and updating of multiple ColorScaleElementSlider widgets.
    
    The SlidersWidget provides a comprehensive interface for managing color scale sliders, 
    offering advanced functionality for color scale customization. Key features include:
    - Dynamic addition and removal of color scale sliders
    - Slider selection and management
    - Support for different widget orientations
    - Interactive slider manipulation
    
    \code{.cpp}
    // Example of creating and using a SlidersWidget
    // Create a shared set of color scale sliders
    SharedColorScaleElementSliders sliders = QSharedPointer<ColorScaleElementSliders>::create();
    
    // Create color scale sliders
    ColorScaleElementSlider* slider1 = new ColorScaleElementSlider();
    slider1->setPosition(0.0);  // Start of the color scale
    
    ColorScaleElementSlider* slider2 = new ColorScaleElementSlider();
    slider2->setPosition(1.0);  // End of the color scale
    
    sliders->addSlider(slider1);
    sliders->addSlider(slider2);
    
    // Create a sliders widget with horizontal orientation
    SlidersWidget* slidersWidget = new SlidersWidget(
        sliders, 
        Qt::Horizontal  // Optional: specify orientation
    );
    
    // Add color scale sliders programmatically
    ColorScaleElementSlider* slider3 = new ColorScaleElementSlider();
    slider3->setPosition(0.5);  // Middle of the color scale
    
    // Add sliders to the widget
    slidersWidget->addSlider(slider3);
    
    // Connect to signals for slider interactions
    QObject::connect(slidersWidget, &SlidersWidget::sliderAdded, 
        [](ColorScaleElementSlider* slider) {
            qDebug() << "New slider added at position: " << slider->position();
        });
    
    QObject::connect(slidersWidget, &SlidersWidget::sliderRemoved, 
        [](ColorScaleElementSlider* slider) {
            qDebug() << "Slider removed from position: " << slider->position();
        });
    \endcode
    
    \note The widget supports both horizontal and vertical slider layouts.
    \see ColorScaleElementSlider, ColorScaleEditorBaseWidget
*/
class CCPLUGIN_LIB_API SlidersWidget : public ColorScaleEditorBaseWidget
{
	Q_OBJECT

public:

	//! Default constructor
	/** \brief Constructs a sliders widget
	 *  \param sliders Shared pointer to the set of color scale element sliders
	 *  \param parent Optional parent widget for ownership and layout
	 *  \param orientation Widget orientation (horizontal or vertical), defaults to horizontal
	 *  
	 *  \details Initializes the sliders widget with the given sliders and orientation.
	 *  The widget will manage and display the provided color scale sliders.
	 *  
	 *  \note If no orientation is specified, the sliders will be laid out horizontally by default.
	 *  
	 *  \see setSliders() to update the color scale sliders after construction
	 */
	SlidersWidget(SharedColorScaleElementSliders sliders, QWidget* parent = nullptr, Qt::Orientation orientation = Qt::Horizontal);

	//! Manually selects a slider
	/** \brief Selects a specific slider in the widget
	 *  \param index Index of the slider to select
	 *  \param silent Optional flag to suppress selection signal emission, defaults to false
	 *  
	 *  \details Changes the current selected slider to the one at the specified index.
	 *  By default, this will emit the sliderSelected signal to notify listeners.
	 *  
	 *  \note If the index is out of range, no selection will occur.
	 *  
	 *  \par Example:
	 *  \code{.cpp}
	 *  SlidersWidget* slidersWidget = ...;
	 *  
	 *  // Select the second slider (index 1)
	 *  slidersWidget->select(1);
	 *  
	 *  // Select a slider without emitting the selection signal
	 *  slidersWidget->select(2, true);
	 *  \endcode
	 */
	void select(int index, bool silent=false);

	//! Adds a new slider widget
	/** \brief Creates and adds a new color scale slider
	 *  \param relativePos Slider position relative to scale boundaries [0.0, 1.0]
	 *  \param color Color of the new slider
	 *  \return Pointer to the newly created ColorScaleElementSlider
	 *  
	 *  \details Adds a new slider to the color scale at the specified relative position 
	 *  and with the given color. The position is normalized between 0.0 and 1.0:
	 *  - 0.0 represents the start of the color scale
	 *  - 1.0 represents the end of the color scale
	 *  
	 *  \note The method takes ownership of the created slider.
	 *  
	 *  \par Example:
	 *  \code{.cpp}
	 *  SlidersWidget* slidersWidget = ...;
	 *  
	 *  // Add a blue slider at the start of the color scale
	 *  ColorScaleElementSlider* startSlider = slidersWidget->addNewSlider(0.0, Qt::blue);
	 *  
	 *  // Add a red slider at the end of the color scale
	 *  ColorScaleElementSlider* endSlider = slidersWidget->addNewSlider(1.0, Qt::red);
	 *  \endcode
	 */
	ColorScaleElementSlider* addNewSlider(double relativePos, QColor color);

	//! Updates slider position
	/** \brief Recalculates and updates the visual position of a specific slider
	 *  \param index Index of the slider to update
	 *  
	 *  \details Triggers a repositioning of the slider based on its current configuration.
	 *  This is useful when the slider's properties have been modified programmatically.
	 *  
	 *  \note If the index is out of range, no update will occur.
	 *  \note This method does not emit any signals by default.
	 */
	void updateSliderPos(int index);

	//! Updates all sliders positions
	/** \brief Recalculates and updates the visual positions of all sliders
	 *  
	 *  \details Triggers a repositioning of all sliders in the widget.
	 *  This is useful when multiple sliders have been modified or the widget's 
	 *  layout has changed.
	 *  
	 *  \note This method updates all sliders without emitting individual signals.
	 */
	void updateAllSlidersPos();

Q_SIGNALS:

	//! Signal emitted when a slider is changed (position or color)
	/** \brief Notifies listeners about modifications to a specific slider
	 *  \param index Index of the modified slider
	 *  
	 *  \details This signal is emitted when a slider's position or color has been changed.
	 *  
	 *  \par Example:
	 *  \code{.cpp}
	 *  SlidersWidget* slidersWidget = ...;
	 *  
	 *  // Connect to slider modification signal
	 *  QObject::connect(slidersWidget, &SlidersWidget::sliderModified,
	 *      [](int index) {
	 *          qDebug() << "Slider at index " << index << " was modified";
	 *      });
	 *  \endcode
	 */
	void sliderModified(int index);

	//! Signal emitted when a slider is selected
	/** \brief Notifies listeners about slider selection changes
	 *  \param index Index of the newly selected slider
	 *  
	 *  \details This signal is emitted when a different slider is manually selected 
	 *  within the widget.
	 *  
	 *  \par Example:
	 *  \code{.cpp}
	 *  SlidersWidget* slidersWidget = ...;
	 *  
	 *  // Connect to slider selection signal
	 *  QObject::connect(slidersWidget, &SlidersWidget::sliderSelected,
	 *      [](int index) {
	 *          qDebug() << "Slider at index " << index << " is now selected";
	 *      });
	 *  \endcode
	 */
	void sliderSelected(int index);

protected:

	//inherited from QWidget
	/** \brief Handles mouse press events for slider interactions
	 *  \param e Pointer to the mouse press event
	 *  
	 *  \details Captures mouse press events to determine the slider 
	 *  that was clicked and to trigger selection or modification.
	 *  
	 *  \note Emits sliderSelected or sliderModified signals as needed.
	 */
	void mousePressEvent(QMouseEvent* e) override;
	/** \brief Handles mouse move events for slider dragging
	 *  \param e Pointer to the mouse move event
	 *  
	 *  \details Captures mouse move events to update the position 
	 *  of the currently selected slider.
	 *  
	 *  \note Emits sliderModified signal when the slider's position changes.
	 */
	void mouseMoveEvent(QMouseEvent* e) override;
	//virtual void mouseReleaseEvent(QMouseEvent* e);
	/** \brief Handles mouse double-click events for slider creation
	 *  \param e Pointer to the mouse double-click event
	 *  
	 *  \details Captures mouse double-click events to create a new 
	 *  slider at the clicked position.
	 *  
	 *  \note Emits sliderAdded signal when a new slider is created.
	 */
	void mouseDoubleClickEvent(QMouseEvent* e) override;
	/** \brief Handles resize events for widget layout updates
	 *  \param e Pointer to the resize event
	 *  
	 *  \details Captures resize events to update the layout of 
	 *  the sliders within the widget.
	 *  
	 *  \note Triggers a repositioning of all sliders.
	 */
	void resizeEvent(QResizeEvent* e) override;

};

//! All sliders labels widget
/** \brief A widget that displays labels for color scale sliders.
    \details This class manages the display of text labels associated with color scale sliders, including text color and precision.
    
    The SliderLabelWidget provides a flexible and customizable way to display labels 
    for color scale sliders. Key features include:
    - Rendering labels for each color scale slider
    - Configurable label text formatting
    - Support for different text colors and precision
    - Adaptable to horizontal and vertical widget orientations
    
    \code{.cpp}
    // Example of creating and configuring a SliderLabelWidget
    // Create a shared set of color scale sliders
    SharedColorScaleElementSliders sliders = QSharedPointer<ColorScaleElementSliders>::create();
    
    // Create color scale sliders
    ColorScaleElementSlider* slider1 = new ColorScaleElementSlider();
    slider1->setPosition(0.0);  // Start of the color scale
    
    ColorScaleElementSlider* slider2 = new ColorScaleElementSlider();
    slider2->setPosition(1.0);  // End of the color scale
    
    sliders->addSlider(slider1);
    sliders->addSlider(slider2);
    
    // Create a horizontal slider label widget
    SliderLabelWidget* labelWidget = new SliderLabelWidget(
        sliders,               // Shared sliders
        Qt::Horizontal,        // Orientation
        2                      // Label precision (decimal places)
    );
    
    // Customize label appearance
    labelWidget->setLabelColor(Qt::black);  // Set label text color
    labelWidget->setLabelPrecision(3);     // Set label precision to 3 decimal places
    
    // Optional: Connect to signals for dynamic label updates
    QObject::connect(labelWidget, &SliderLabelWidget::labelUpdated,
        [](int index, double value) {
            qDebug() << "Label at index " << index << " updated to: " << value;
        });
    \endcode
    
    \note The widget supports both horizontal and vertical label layouts.
    \see ColorScaleElementSliders, ColorScaleEditorBaseWidget
*/
class CCPLUGIN_LIB_API SliderLabelWidget : public ColorScaleEditorBaseWidget
{
	Q_OBJECT

public:

	//! Default constructor
	/** \brief Constructs a slider label widget
	 *  \param sliders Shared pointer to the set of color scale element sliders
	 *  \param parent Optional parent widget for ownership and layout
	 *  \param orientation Widget orientation (horizontal or vertical), defaults to horizontal
	 *  
	 *  \details Initializes the slider label widget with specified sliders and orientation.
	 *  The widget will display labels for the provided color scale sliders.
	 *  
	 *  \note If no orientation is specified, labels will be laid out horizontally by default.
	 *  
	 *  \par Example:
	 *  \code{.cpp}
	 *  // Create a shared set of sliders
	 *  SharedColorScaleElementSliders sliders = QSharedPointer<ColorScaleElementSliders>::create();
	 *  
	 *  // Create a horizontal slider label widget
	 *  SliderLabelWidget* labelWidget = new SliderLabelWidget(sliders);
	 *  
	 *  // Create a vertical slider label widget
	 *  SliderLabelWidget* verticalLabelWidget = new SliderLabelWidget(
	 *      sliders, 
	 *      nullptr,             // No parent
	 *      Qt::Vertical         // Vertical orientation
	 *  );
	 *  \endcode
	 */
	SliderLabelWidget(SharedColorScaleElementSliders sliders, QWidget* parent = nullptr, Qt::Orientation orientation = Qt::Horizontal);

	//! Sets text color
	/** \brief Configures the color of slider labels
	 *  \param color Desired text color for labels
	 *  
	 *  \details Sets the text color for all slider labels in the widget.
	 *  This allows for visual customization and improved readability.
	 *  
	 *  \note The color change applies to all existing and future labels.
	 *  \note This is an inline method that directly sets the text color member variable.
	 *  
	 *  \par Example:
	 *  \code{.cpp}
	 *  SliderLabelWidget* labelWidget = ...;
	 *  
	 *  // Set label text color to dark gray
	 *  labelWidget->setTextColor(QColor(64, 64, 64));
	 *  
	 *  // Set label text color using predefined Qt colors
	 *  labelWidget->setTextColor(Qt::blue);
	 *  \endcode
	 */
	inline void setTextColor(QColor color) { m_textColor = color; }

	//! Sets displayed numbers precision
	/** \brief Configures the number of decimal places for label values
	 *  \param precision Number of decimal places to display
	 *  
	 *  \details Controls the formatting of numeric values in slider labels.
	 *  Allows fine-tuning of label readability and information density.
	 *  
	 *  Precision constraints:
	 *  - Minimum value: 0 (whole numbers)
	 *  - Maximum recommended value: 6 (to prevent excessive decimal display)
	 *  
	 *  \note This is an inline method that directly sets the precision member variable.
	 *  \note Changing precision may require a manual widget update.
	 *  
	 *  \par Example:
	 *  \code{.cpp}
	 *  SliderLabelWidget* labelWidget = ...;
	 *  
	 *  // Display labels with whole numbers
	 *  labelWidget->setPrecision(0);  // 1 instead of 1.000
	 *  
	 *  // Display labels with 3 decimal places
	 *  labelWidget->setPrecision(3);  // 1.500 instead of 1.5
	 *  \endcode
	 */
	inline void setPrecision(int precision) { m_precision = precision; }

protected:

	//inherited from QWidget
	/** \brief Handles the paint event for rendering the slider labels
	 *  \param e Pointer to the paint event
	 *  
	 *  \details Overrides the default QWidget paint event to draw 
	 *  text labels for color scale sliders. The rendering takes into account:
	 *  - Current text color (m_textColor)
	 *  - Label precision setting (m_precision)
	 *  - Widget orientation
	 *  
	 *  \note This method is called automatically by Qt's event system 
	 *  when the widget needs to be redrawn.
	 */
	void paintEvent(QPaintEvent* e) override;

	//! Text color
	/** \brief Stores the color used for rendering slider labels
	 *  \details Member variable that defines the text color for all 
	 *  labels in the slider widget.
	 *  
	 *  \note Can be modified using setTextColor() method
	 */
	QColor m_textColor;

	//! Precision
	/** \brief Determines the number of decimal places for label values
	 *  \details Controls the formatting of numeric values displayed 
	 *  in slider labels.
	 *  
	 *  \note Can be modified using setPrecision() method
	 *  \note Typical range: 0-6 decimal places
	 */
	int m_precision;
};

//! Color scale editor dialog
/** \brief A dialog widget for editing color scales.
    \details This class provides functionality to manage color scale steps, including adding, deleting, and modifying sliders.
    
    The ccColorScaleEditorWidget offers a comprehensive and interactive interface 
    for creating and manipulating color scales. Key features include:
    - Dynamic color scale step management
    - Visual color bar representation
    - Intuitive slider interactions
    - Customizable label display
    - Color scale import and export capabilities
    
    Core components:
    - Color Bar: Visual representation of the color gradient
    - Sliders Widget: Allows precise step positioning and color selection
    - Labels Widget: Displays numeric values for each color scale step
    
    \code{.cpp}
    // Example of creating and using a color scale editor widget
    ccColorScaleEditorWidget* editor = new ccColorScaleEditorWidget();
    
    // Configure initial color scale steps
    editor->setStepColor(0, Qt::blue);   // First step color
    editor->setStepColor(1, Qt::red);    // Second step color
    
    // Set step positions
    editor->setStepRelativePosition(0, 0.0);  // First step at start
    editor->setStepRelativePosition(1, 1.0);  // Second step at end
    
    // Add a new intermediate step
    ColorScaleElementSlider* midStep = editor->addNewSlider(0.5, Qt::green);
    
    // Customize label appearance
    editor->showLabels(true);
    editor->setLabelColor(Qt::black);
    editor->setLabelPrecision(2);
    
    // Connect to signals for tracking changes
    QObject::connect(editor, &ccColorScaleEditorWidget::stepModified,
        [](int index) {
            qDebug() << "Step at index " << index << " was modified";
        });
    
    // Export the color scale for later use
    ccColorScale::Shared exportedScale;
    editor->exportColorScale(exportedScale);
    
    // Import a previously saved color scale
    ccColorScale::Shared importedScale = ...;
    editor->importColorScale(importedScale);
    \endcode
    
    \note The widget supports both horizontal and vertical color scale layouts.
    \note Provides a user-friendly interface for complex color scale editing.
    
    \see ColorScaleElementSlider
    \see ColorBarWidget
    \see SlidersWidget
    \see SliderLabelWidget
*/
class CCPLUGIN_LIB_API ccColorScaleEditorWidget : public ColorScaleEditorBaseWidget
{
	Q_OBJECT

public:

	//! Default constructor
	/** \brief Constructs a color scale editor widget with flexible orientation
	 *  \param parent Optional parent widget for memory management
	 *  \param orientation Defines the layout direction of the color scale (horizontal or vertical)
	 *  
	 *  \details Creates a customizable color scale editor widget that can be 
	 *  integrated into various UI layouts. The widget supports two primary 
	 *  orientations:
	 *  - Horizontal: Color scale spans from left to right (default)
	 *  - Vertical: Color scale spans from top to bottom
	 *  
	 *  Initialization steps:
	 *  - Sets up internal widgets (color bar, sliders, labels)
	 *  - Configures default color scale parameters
	 *  - Applies specified orientation
	 *  
	 *  \note If no parent is specified, the widget becomes a top-level window
	 *  
	 *  \par Examples:
	 *  \code{.cpp}
	 *  // Create a horizontal color scale editor (default)
	 *  ccColorScaleEditorWidget* horizontalEditor = 
	 *      new ccColorScaleEditorWidget();
	 *  
	 *  // Create a vertical color scale editor with a parent
	 *  QWidget* parentWidget = ...;
	 *  ccColorScaleEditorWidget* verticalEditor = 
	 *      new ccColorScaleEditorWidget(parentWidget, Qt::Vertical);
	 *  
	 *  // Embedding in a layout
	 *  QVBoxLayout* layout = new QVBoxLayout();
	 *  layout->addWidget(verticalEditor);
	 *  \endcode
	 *  
	 *  \see ColorScaleElementSliders
	 *  \see Qt::Orientation
	 */
	ccColorScaleEditorWidget(QWidget* parent = nullptr, Qt::Orientation orientation = Qt::Horizontal);

	//! Destructor
	~ccColorScaleEditorWidget() override = default;

	//! Returns the current number of color scale steps
	/** \brief Retrieves the total number of color scale steps
	 *  \returns Number of color scale steps, or 0 if no sliders are configured
	 *  
	 *  \details Provides a quick way to determine the complexity of the 
	 *  current color scale. Returns the number of individual color steps 
	 *  that define the gradient.
	 *  
	 *  Thread-safety: Safe to call from any thread
	 *  
	 *  \note Returns 0 if the slider collection is not initialized
	 *  
	 *  \par Examples:
	 *  \code{.cpp}
	 *  ccColorScaleEditorWidget* editor = new ccColorScaleEditorWidget();
	 *  
	 *  // Initial step count (typically 2)
	 *  int initialSteps = editor->getStepCount();
	 *  qDebug() << "Initial color scale steps: " << initialSteps;
	 *  
	 *  // Add a new step
	 *  editor->addNewSlider(0.5, Qt::green);
	 *  
	 *  // Updated step count
	 *  int updatedSteps = editor->getStepCount();
	 *  qDebug() << "Updated color scale steps: " << updatedSteps;
	 *  
	 *  // Checking step count before operations
	 *  if (editor->getStepCount() > 2) {
	 *      // Perform advanced color scale manipulation
	 *  }
	 *  \endcode
	 *  
	 *  \see addNewSlider()
	 *  \see removeSlider()
	 */
	inline int getStepCount() const { return (m_sliders ? m_sliders->size() : 0); }

	//! Returns a given slider (pointer on)
	/** \brief Retrieves a specific color scale step by its index
	 *  \param index Zero-based index of the color scale step
	 *  \returns Pointer to the ColorScaleElementSlider at the specified index, 
	 *           or nullptr if index is out of bounds or sliders are not initialized
	 *  
	 *  \details Provides direct access to individual color scale steps.
	 *  Allows inspection and manipulation of specific slider properties.
	 *  
	 *  Key behaviors:
	 *  - Returns nullptr if index is invalid
	 *  - Does not modify the underlying color scale
	 *  - Provides read-only access to slider details
	 *  
	 *  \warning Returned pointer should not be deleted manually
	 *  
	 *  \par Examples:
	 *  \code{.cpp}
	 *  ccColorScaleEditorWidget* editor = new ccColorScaleEditorWidget();
	 *  
	 *  // Get the first color scale step
	 *  const ColorScaleElementSlider* firstStep = editor->getStep(0);
	 *  if (firstStep) {
	 *      QColor stepColor = firstStep->color();
	 *      double stepPosition = firstStep->position();
	 *      
	 *      qDebug() << "First step color: " << stepColor.name();
	 *      qDebug() << "First step position: " << stepPosition;
	 *  }
	 *  
	 *  // Safely check step details
	 *  for (int i = 0; i < editor->getStepCount(); ++i) {
	 *      const ColorScaleElementSlider* step = editor->getStep(i);
	 *      if (step) {
	 *          // Perform operations on each step
	 *      }
	 *  }
	 *  \endcode
	 *  
	 *  \note Complexity: O(1) access time
	 *  \see getStepCount()
	 *  \see ColorScaleElementSlider
	 */
	inline const ColorScaleElementSlider* getStep(int index) { return m_sliders ? m_sliders->elements().at(index) : nullptr; }

	//! Sets a given slider color
	/** \brief Modifies the color of a specific color scale step
	 *  \param index Zero-based index of the color scale step to modify
	 *  \param color New color to assign to the specified step
	 *  
	 *  \details Updates the color of an individual step in the color scale.
	 *  Triggers a visual update of the color scale widget.
	 *  
	 *  Key behaviors:
	 *  - Silently fails if index is out of bounds
	 *  - Immediately updates the color scale visualization
	 *  - Emits internal signals to refresh the widget
	 *  
	 *  \par Examples:
	 *  \code{.cpp}
	 *  ccColorScaleEditorWidget* editor = new ccColorScaleEditorWidget();
	 *  
	 *  // Modify first step color
	 *  editor->setStepColor(0, Qt::blue);
	 *  
	 *  // Create a gradient-like color scale
	 *  editor->setStepColor(0, QColor(0, 0, 255));     // Deep Blue
	 *  editor->setStepColor(1, QColor(255, 0, 0));     // Deep Red
	 *  
	 *  // Add and color an intermediate step
	 *  editor->addNewSlider(0.5, Qt::green);
	 *  editor->setStepColor(2, QColor(0, 255, 0));     // Bright Green
	 *  
	 *  // Safe color modification with bounds checking
	 *  if (editor->getStepCount() > 2) {
	 *      editor->setStepColor(2, QColor(255, 255, 0)); // Yellow
	 *  }
	 *  \endcode
	 *  
	 *  \note Triggers a repaint of the color scale widget
	 *  \see getStep()
	 *  \see addNewSlider()
	 */
	void setStepColor(int index, QColor color);

	//! Sets a given slider relative position
	/** \brief Adjusts the position of a specific color scale step
	 *  \param index Zero-based index of the color scale step to modify
	 *  \param relativePos New relative position for the step (0.0 to 1.0)
	 *  
	 *  \details Moves a color scale step to a new position along the gradient.
	 *  The relative position is normalized between 0.0 (start) and 1.0 (end).
	 *  
	 *  Key behaviors:
	 *  - Validates input range (0.0 to 1.0)
	 *  - Prevents overlapping step positions
	 *  - Triggers visual update of color scale
	 *  
	 *  Position constraints:
	 *  - First and last steps typically at 0.0 and 1.0
	 *  - Intermediate steps between 0.0 and 1.0
	 *  
	 *  \par Examples:
	 *  \code{.cpp}
	 *  ccColorScaleEditorWidget* editor = new ccColorScaleEditorWidget();
	 *  
	 *  // Position first step at the start of the gradient
	 *  editor->setStepRelativePosition(0, 0.0);
	 *  
	 *  // Position last step at the end of the gradient
	 *  editor->setStepRelativePosition(1, 1.0);
	 *  
	 *  // Add and position an intermediate step
	 *  editor->addNewSlider(0.5, Qt::green);
	 *  editor->setStepRelativePosition(2, 0.5);  // Middle of the gradient
	 *  
	 *  // Safe positioning with bounds checking
	 *  if (editor->getStepCount() > 2) {
	 *      // Ensure position is within valid range
	 *      double newPos = std::clamp(0.75, 0.0, 1.0);
	 *      editor->setStepRelativePosition(2, newPos);
	 *  }
	 *  \endcode
	 *  
	 *  \warning 
	 *  - Silently fails if index is out of bounds
	 *  - Does not prevent potential position conflicts
	 *  
	 *  \note Triggers a repaint of the color scale widget
	 *  \see getStep()
	 *  \see addNewSlider()
	 *  \see setStepColor()
	 */
	void setStepRelativePosition(int index, double relativePos);

	//! Returns currently selected step index
	inline int getSelectedStepIndex() const { return m_sliders ? m_sliders->selected() : -1; }

	//! Sets currently selected step index
	/** \brief Programmatically selects a specific color scale step
	 *  \param index Zero-based index of the step to select
	 *  \param silent Optional flag to suppress selection change notifications
	 *  
	 *  \details Allows manual selection of a color scale step, 
	 *  with optional control over notification behavior.
	 *  
	 *  Key behaviors:
	 *  - Validates index against available steps
	 *  - Updates widget focus and visual selection
	 *  - Optionally suppresses selection change notifications
	 *  
	 *  Selection scenarios:
	 *  - Programmatic step selection
	 *  - Preparing for batch editing
	 *  - Synchronizing selection across multiple widgets
	 *  
	 *  Notification control:
	 *  - When `silent` is false (default):
	 *    * Emits selection change signals
	 *    * Triggers visual focus update
	 *  - When `silent` is true:
	 *    * Suppresses selection change signals
	 *    * Performs selection without visual notification
	 *  
	 *  \par Examples:
	 *  \code{.cpp}
	 *  ccColorScaleEditorWidget* editor = new ccColorScaleEditorWidget();
	 *  
	 *  // Select the first step
	 *  editor->setSelectedStepIndex(0);
	 *  
	 *  // Add an intermediate step
	 *  editor->addNewSlider(0.5, Qt::green);
	 *  
	 *  // Select the new intermediate step
	 *  editor->setSelectedStepIndex(2);
	 *  
	 *  // Silent selection (no signals or visual updates)
	 *  editor->setSelectedStepIndex(1, true);
	 *  
	 *  // Safe selection with bounds checking
	 *  if (editor->getStepCount() > 2) {
	 *      editor->setSelectedStepIndex(2);
	 *  }
	 *  
	 *  // Connecting to selection change signals
	 *  QObject::connect(editor, &ccColorScaleEditorWidget::stepSelectionChanged,
	 *      [](int newIndex) {
	 *          qDebug() << "Selected step changed to: " << newIndex;
	 *      });
	 *  \endcode
	 *  
	 *  \warning 
	 *  - Silently fails if index is out of bounds
	 *  - Does not modify the step itself, only selection
	 *  
	 *  \note Potential widget repaint and focus changes
	 *  \see getSelectedStepIndex()
	 *  \see getStepCount()
	 *  \see stepSelectionChanged signal
	 */
	void setSelectedStepIndex(int index, bool silent=false);

	//! Deletes a given step
	/** \brief Removes a specific color scale step from the gradient
	 *  \param index Zero-based index of the color scale step to delete
	 *  
	 *  \details Removes an intermediate color scale step, 
	 *  maintaining the integrity of the color gradient.
	 *  
	 *  Key restrictions and behaviors:
	 *  - Cannot delete first and last steps (boundary steps)
	 *  - Silently fails if index is out of bounds
	 *  - Prevents reduction of color scale to less than two steps
	 *  - Triggers visual update of color scale
	 *  
	 *  Step deletion scenarios:
	 *  - Removing intermediate color transition points
	 *  - Simplifying complex color gradients
	 *  - Cleaning up unnecessary steps
	 *  
	 *  \par Examples:
	 *  \code{.cpp}
	 *  ccColorScaleEditorWidget* editor = new ccColorScaleEditorWidget();
	 *  
	 *  // Add an intermediate step
	 *  editor->addNewSlider(0.5, Qt::green);
	 *  
	 *  // Delete the intermediate step
	 *  editor->deleteStep(2);
	 *  
	 *  // Safe step deletion with bounds checking
	 *  if (editor->getStepCount() > 2) {
	 *      // Delete the third step (index 2)
	 *      editor->deleteStep(2);
	 *  }
	 *  
	 *  // Attempt to delete boundary steps (will be ignored)
	 *  editor->deleteStep(0);  // First step
	 *  editor->deleteStep(1);  // Last step
	 *  
	 *  // Programmatic step management
	 *  for (int i = editor->getStepCount() - 1; i > 1; --i) {
	 *      editor->deleteStep(i);  // Remove extra steps from the end
	 *  }
	 *  \endcode
	 *  
	 *  \warning 
	 *  - Attempting to delete first or last steps will be silently ignored
	 *  - Ensures at least two steps remain in the color scale
	 *  
	 *  \note Triggers a repaint of the color scale widget
	 *  \see addNewSlider()
	 *  \see getStepCount()
	 *  \see setStepColor()
	 */
	void deleteStep(int index);

	//! Exports the current color scale
	void exportColorScale(ccColorScale::Shared& destScale) const;

	//! Imports the current color scale
	void importColorScale(ccColorScale::Shared scale);

	//! Sets whether to show the color elements labels or not
	void showLabels(bool state);

	//! Sets the labels color
	void setLabelColor(QColor color);

	//! Sets the labels precision
	void setLabelPrecision(int precision);

	//inherited from ColorScaleEditorBaseWidget
	void setSliders(SharedColorScaleElementSliders sliders) override;

Q_SIGNALS:

	//! Signal emitted when a slider is selected
	void stepSelected(int index);

	//! Signal emitted when a slider is modified
	void stepModified(int index);

protected:

	//! Slot called when a 'point' is clicked on the color bar 
	void onPointClicked(double relativePos);

	//! Slot called when a slider is moved or its color is changed
	void onSliderModified(int sliderIndex);

	//! Slot called when a slider is selected
	void onSliderSelected(int sliderIndex);

protected:

	//! Associated color bar
	ColorBarWidget* m_colorBarWidget;

	//! Associated sliders widget
	SlidersWidget* m_slidersWidget;

	//! Associated (sliders) labels widget
	SliderLabelWidget* m_labelsWidget;

};
