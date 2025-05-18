#include "../include/qE57ImageExtractorAction"
#include "ccMainAppInterface.h"
#include <iostream>

#include <QMessageBox>
#include <QFileDialog>
#include <ccImage.h>
#include <unordered_set>
#include <unordered_map>
#include <future>
#include <mutex>

namespace qE57ImageExtractorAction
{
	void extractAndSaveImages( ccMainAppInterface *appInterface, const ccHObject::Container &selectedEntities )
	{
		if ( appInterface == nullptr )
		{
			// The application interface should have already been initialized when the plugin is loaded
			Q_ASSERT( false );
		}

        if (selectedEntities.empty())
        {
            QMessageBox::warning(nullptr, "Warning", "No images selected");
        }

        auto saveDirectory = QString();
        while (saveDirectory.isEmpty())
        {
            saveDirectory = QFileDialog::getExistingDirectory(nullptr, "Choose destination directory", {},
                                                              QFileDialog::ShowDirsOnly |
                                                              QFileDialog::DontResolveSymlinks);
        }
        appInterface->dispToConsole("Destination directory: " + saveDirectory, ccMainAppInterface::STD_CONSOLE_MESSAGE);

        int numSavedImages = saveImages(saveDirectory, selectedEntities);
        appInterface->dispToConsole(QString::fromStdString("Images saved: " + std::to_string(numSavedImages)), ccMainAppInterface::STD_CONSOLE_MESSAGE);
	}

    int saveImages(const QString& directory, const ccHObject::Container& selectedEntities)
    {
        auto savedImageCount {0};
        std::unordered_set<ccImage*> imagesToSave{};
        std::unordered_map<QString, int> imageGroupCount{};
        std::mutex imageGroupCountMutex;
        std::vector<std::future<void>> futures{};

        // Search all selected entities for images
        for (const auto& entity : selectedEntities)
        {
            // If the current entity is of type image, cast to QImage and store in imagesToSave
            if (entity->isKindOf(CC_TYPES::IMAGE))
            {
                auto image = ccHObjectCaster::ToImage(entity);
                if (image)
                {
                    imagesToSave.insert(image);
                }
            }
            // If the current entity is of type Point Cloud, search children for images
            else if (entity->isKindOf(CC_TYPES::POINT_CLOUD))
            {
                for (int i = 0; i < entity->getChildrenNumber(); ++i)
                {
                    auto child = entity->getChild(i);

                    // In case image was found among the children, check if the image is already stored, if not store it
                    // This is necessary for cases when both the point cloud and the child images were selected by the user
                    if (child && child->isKindOf(CC_TYPES::IMAGE))
                    {
                        auto image = ccHObjectCaster::ToImage(child);
                        if (image)
                        {
                            imagesToSave.insert(image);
                        }
                    }
                }
            }
        }

        auto saveCurrentImage = [&](ccImage* image)
        {
            if (image)
            {
                QString filename;
                {
                    std::lock_guard<std::mutex> lock(imageGroupCountMutex);
                    filename = directory + OS_FILE_SEP + image->getName() + "_" + QString::fromStdString(std::to_string(imageGroupCount[image->getName()]++)) + ".png";
                }
                if (image->data().save(filename))
                {
                    std::lock_guard<std::mutex> lock(imageGroupCountMutex);
                    ++savedImageCount;
                }
            }
        };
        
        // Save the images to the provided directory
        for (const auto& image : imagesToSave)
        {
            futures.push_back(std::async(std::launch::async, saveCurrentImage, image));
        }

        // Sync finished threads
        for (const auto& future : futures)
        {
            future.wait();
        }
        return savedImageCount;
    }
}
