// Copyright 2015 TIER IV, Inc. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// Copyright (c) 2014, JSK Lab
// All rights reserved.
//
// Software License Agreement (BSD License)
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//
//  * Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//  * Redistributions in binary form must reproduce the above
//    copyright notice, this list of conditions and the following
//    disclaimer in the documentation and/or other materials provided
//    with the distribution.
//  * Neither the name of {copyright_holder} nor the names of its
//    contributors may be used to endorse or promote products derived
//    from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
// FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
// COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
// INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
// BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
// LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
// ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.S SOFTWARE, EVEN IF ADVISED OF THE
//  POSSIBILITY OF SUCH DAMAGE.

#ifndef JSK_OVERLAY_UTILS_HPP_
#define JSK_OVERLAY_UTILS_HPP_

#include <OgreHardwarePixelBuffer.h>
#include <OgreMaterialManager.h>
#include <OgreSceneManager.h>
#include <OgreTechnique.h>
#include <OgreTexture.h>
#include <OgreTextureManager.h>

#include <memory>
#include <string>
// see OGRE/OgrePrerequisites.h
// #define OGRE_VERSION
// ((OGRE_VERSION_MAJOR << 16) | (OGRE_VERSION_MINOR << 8) | OGRE_VERSION_PATCH)
#if OGRE_VERSION < ((1 << 16) | (9 << 8) | 0)
#include <OGRE/OgreOverlayContainer.h>
#include <OGRE/OgreOverlayElement.h>
#include <OGRE/OgreOverlayManager.h>
#include <OGRE/OgrePanelOverlayElement.h>
#else
#include <Overlay/OgreOverlay.h>
#include <Overlay/OgreOverlayContainer.h>
#include <Overlay/OgreOverlayElement.h>
#include <Overlay/OgreOverlayManager.h>
#include <Overlay/OgrePanelOverlayElement.h>
#endif

#include <QColor>
#include <QCursor>
#include <QImage>
#include <QVariant>
#include <rclcpp/rclcpp.hpp>
#include <rviz_rendering/render_system.hpp>

namespace jsk_rviz_plugins
{
class OverlayObject;

/**
 * @class ScopedPixelBuffer
 * @brief Manages a scoped pixel buffer for RViz overlay objects.
 *
 * This class is designed to manage the lifetime of an Ogre::HardwarePixelBufferSharedPtr.
 * It locks the pixel buffer on creation and unlocks it on destruction, providing a
 * convenient way to manipulate pixel buffers within a scoped lifetime. It also offers
 * methods to convert the buffer into a QImage format.
 */
class ScopedPixelBuffer
{
public:
  /**
   * @brief Constructs a ScopedPixelBuffer object and locks the pixel buffer.
   * @param pixel_buffer The Ogre hardware pixel buffer to be managed.
   */
  explicit ScopedPixelBuffer(Ogre::HardwarePixelBufferSharedPtr pixel_buffer);

  /**
   * @brief Destructor that unlocks the pixel buffer.
   */
  virtual ~ScopedPixelBuffer();

  /**
   * @brief Gets the underlying Ogre hardware pixel buffer.
   * @return Shared pointer to the Ogre::HardwarePixelBuffer.
   */
  virtual Ogre::HardwarePixelBufferSharedPtr getPixelBuffer();

  /**
   * @brief Converts the pixel buffer to a QImage.
   * @param width Width of the QImage.
   * @param height Height of the QImage.
   * @return QImage representation of the pixel buffer.
   */
  virtual QImage getQImage(unsigned int width, unsigned int height);

  /**
   * @brief Converts the pixel buffer of an overlay object to a QImage.
   * @param overlay Reference to the OverlayObject.
   * @return QImage representation of the overlay object's pixel buffer.
   */
  virtual QImage getQImage(OverlayObject & overlay);

  /**
   * @brief Converts the pixel buffer to a QImage with a specified background color.
   * @param width Width of the QImage.
   * @param height Height of the QImage.
   * @param bg_color Background color for the QImage.
   * @return QImage representation of the pixel buffer with background color.
   */
  virtual QImage getQImage(unsigned int width, unsigned int height, QColor & bg_color);

  /**
   * @brief Converts the pixel buffer of an overlay object to a QImage with a specified background color.
   * @param overlay Reference to the OverlayObject.
   * @param bg_color Background color for the QImage.
   * @return QImage representation of the overlay object's pixel buffer with background color.
   */
  virtual QImage getQImage(OverlayObject & overlay, QColor & bg_color);

protected:
  Ogre::HardwarePixelBufferSharedPtr
    pixel_buffer_;  ///< Shared pointer to the pixel buffer managed by this class.

private:
};

/**
 * @brief This is a class for put overlay object on rviz 3D panel.
 * This class suppose to be instantiated in onInitialize method of rviz::Display class.
 */
class OverlayObject
{
public:
  typedef std::shared_ptr<OverlayObject> Ptr;

  OverlayObject(
    Ogre::SceneManager * manager, const rclcpp::Logger & logger, const std::string & name);
  virtual ~OverlayObject();

  virtual std::string getName();
  virtual void hide();
  virtual void show();
  virtual bool isTextureReady();
  virtual void updateTextureSize(unsigned int width, unsigned int height);
  virtual ScopedPixelBuffer getBuffer();
  virtual void setPosition(double left, double top);
  virtual void setDimensions(double width, double height);
  virtual bool isVisible();
  virtual unsigned int getTextureWidth();
  virtual unsigned int getTextureHeight();

protected:
  const std::string name_;
  const rclcpp::Logger logger_;
  Ogre::Overlay * overlay_;
  Ogre::PanelOverlayElement * panel_;
  Ogre::MaterialPtr panel_material_;
  Ogre::TexturePtr texture_;

private:
};
}  // namespace jsk_rviz_plugins

#endif  // JSK_OVERLAY_UTILS_HPP_