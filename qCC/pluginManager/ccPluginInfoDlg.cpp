// ##########################################################################
// #                                                                        #
// #                              CLOUDCOMPARE                              #
// #                                                                        #
// #  This program is free software; you can redistribute it and/or modify  #
// #  it under the terms of the GNU General Public License as published by  #
// #  the Free Software Foundation; version 2 or later of the License.      #
// #                                                                        #
// #  This program is distributed in the hope that it will be useful,       #
// #  but WITHOUT ANY WARRANTY; without even the implied warranty of        #
// #  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the          #
// #  GNU General Public License for more details.                          #
// #                                                                        #
// #          COPYRIGHT: CloudCompare project                               #
// #                                                                        #
// ##########################################################################

#include "ccPluginInfoDlg.h"

#include "ccPluginManager.h"
#include "ccStdPluginInterface.h"
#include "ui_ccPluginInfoDlg.h"

#include <QDebug>
#include <QDir>
#include <QSortFilterProxyModel>
#include <QStandardItemModel>

static QString sFormatReferenceList(const ccPluginInterface::ReferenceList& list)
{
	const QString linkFormat(" <a href=\"%1\" style=\"text-decoration:none\">&#x1F517;</a>");
	QString       formattedText;
	int           referenceNum = 1;

	for (const ccPluginInterface::Reference& reference : list)
	{
		formattedText += QStringLiteral("%1. ").arg(QString::number(referenceNum++));

		formattedText += reference.article;

		if (!reference.url.isEmpty())
		{
			formattedText += linkFormat.arg(reference.url);
		}

		formattedText += QStringLiteral("<br/>");
	}

	return formattedText;
}

static QString sFormatContactList(const ccPluginInterface::ContactList& list, const QString& pluginName)
{
	const QString emailFormat("&lt;<a href=\"mailto:%1?Subject=CloudCompare %2\">%1</a>&gt;");
	QString       formattedText;

	for (const ccPluginInterface::Contact& contact : list)
	{
		formattedText += contact.name;

		if (!contact.email.isEmpty())
		{
			formattedText += emailFormat.arg(contact.email, pluginName);
		}

		formattedText += QStringLiteral("<br/>");
	}

	return formattedText;
}

namespace
{
	class _Icons
	{
	  public:
		static QIcon sGetIcon(CC_PLUGIN_TYPE inPluginType)
		{
			if (sIconMap.empty())
			{
				_init();
			}

			return sIconMap[inPluginType];
		}

	  private:
		static void _init()
		{
			if (!sIconMap.empty())
			{
				return;
			}

			sIconMap[CC_STD_PLUGIN]       = QIcon(":/CC/pluginManager/images/std_plugin.png");
			sIconMap[CC_GL_FILTER_PLUGIN] = QIcon(":/CC/pluginManager/images/gl_plugin.png");
			sIconMap[CC_IO_FILTER_PLUGIN] = QIcon(":/CC/pluginManager/images/io_plugin.png");
		}

		static QMap<CC_PLUGIN_TYPE, QIcon> sIconMap;
	};

	QMap<CC_PLUGIN_TYPE, QIcon> _Icons::sIconMap;
} // namespace

ccPluginInfoDlg::ccPluginInfoDlg(QWidget* parent)
    : QDialog(parent)
    , m_ui(std::make_unique<Ui::ccPluginInfoDlg>())
    , m_ProxyModel(new QSortFilterProxyModel(this))
    , m_ItemModel(new QStandardItemModel(this))
{
	m_ui->setupUi(this);

	setWindowTitle(tr("About Plugins"));

	m_ui->mWarningLabel->setText(tr("Enabling/disabling plugins will take effect next time you run %1").arg(QApplication::applicationName()));
	m_ui->mWarningLabel->setStyleSheet(QStringLiteral("QLabel { background-color : #FFFF99; color : black; }"));
	m_ui->mWarningLabel->hide();

	m_ui->mSearchLineEdit->setStyleSheet("QLineEdit, QLineEdit:focus { border: none; }");
	m_ui->mSearchLineEdit->setAttribute(Qt::WA_MacShowFocusRect, false);

	m_ProxyModel->setFilterCaseSensitivity(Qt::CaseInsensitive);
	m_ProxyModel->setSourceModel(m_ItemModel);

	m_ui->mPluginListView->setModel(m_ProxyModel);
	m_ui->mPluginListView->setFocus();

	connect(m_ui->mSearchLineEdit, &QLineEdit::textEdited, m_ProxyModel, &QSortFilterProxyModel::setFilterFixedString);

	connect(m_ui->mPluginListView->selectionModel(), &QItemSelectionModel::currentChanged, this, &ccPluginInfoDlg::selectionChanged);
}

ccPluginInfoDlg::~ccPluginInfoDlg() = default;

void ccPluginInfoDlg::setPluginPaths(const QStringList& pluginPaths)
{
	QString paths;

	for (const QString& path : pluginPaths)
	{
		paths += QDir::toNativeSeparators(path);
		paths += QStringLiteral("\n");
	}

	m_ui->mPluginPathTextEdit->setText(paths);
}

void ccPluginInfoDlg::setPluginList(const QList<ccPluginInterface*>& pluginList)
{
	m_ItemModel->clear();
	m_ItemModel->setRowCount(pluginList.count());
	m_ItemModel->setColumnCount(1);

	int row = 0;

	for (const ccPluginInterface* plugin : pluginList)
	{
		auto name    = plugin->getName();
		auto tooltip = tr("%1 Plugin").arg(plugin->getName());

		if (plugin->isCore())
		{
			tooltip += tr(" (core)");
		}
		else
		{
			name += " 👽";
			tooltip += tr(" (3rd Party)");
		}

		QStandardItem* item = new QStandardItem(name);

		item->setCheckable(true);

		if (ccPluginManager::Get().isEnabled(plugin))
		{
			item->setCheckState(Qt::Checked);
		}

		item->setData(QVariant::fromValue(plugin), PLUGIN_PTR);
		item->setIcon(_Icons::sGetIcon(plugin->getType()));
		item->setToolTip(tooltip);

		m_ItemModel->setItem(row, 0, item);

		++row;
	}

	if (!pluginList.empty())
	{
		m_ItemModel->sort(0);

		QModelIndex index = m_ItemModel->index(0, 0);

		m_ui->mPluginListView->setCurrentIndex(index);
	}

	connect(m_ItemModel, &QStandardItemModel::itemChanged, this, &ccPluginInfoDlg::itemChanged);
}

const ccPluginInterface* ccPluginInfoDlg::pluginFromItemData(const QStandardItem* item) const
{
	return item->data(PLUGIN_PTR).value<const ccPluginInterface*>();
	;
}

void ccPluginInfoDlg::selectionChanged(const QModelIndex& current, const QModelIndex& previous)
{
	Q_UNUSED(previous);

	auto sourceItem = m_ProxyModel->mapToSource(current);
	auto item       = m_ItemModel->itemFromIndex(sourceItem);

	if (item == nullptr)
	{
		// This happens if we are filtering and there are no results
		updatePluginInfo(nullptr);
		return;
	}

	auto plugin = pluginFromItemData(item);

	updatePluginInfo(plugin);
}

void ccPluginInfoDlg::itemChanged(QStandardItem* item)
{
	bool checked = item->checkState() == Qt::Checked;
	auto plugin  = pluginFromItemData(item);

	if (plugin != nullptr)
	{
		ccPluginManager::Get().setPluginEnabled(plugin, checked);

		if (m_ui->mWarningLabel->isHidden())
		{
			ccLog::Warning(m_ui->mWarningLabel->text());

			m_ui->mWarningLabel->show();
		}
	}
}

void ccPluginInfoDlg::updatePluginInfo(const ccPluginInterface* plugin)
{
	if (plugin == nullptr)
	{
		m_ui->mIcon->setPixmap(QPixmap());
		m_ui->mNameLabel->setText(tr("(No plugin selected)"));
		m_ui->mDescriptionTextEdit->clear();
		m_ui->mReferencesTextBrowser->clear();
		m_ui->mAuthorsTextBrowser->clear();
		m_ui->mMaintainerTextBrowser->clear();
		return;
	}

	const QSize iconSize(64, 64);

	QPixmap iconPixmap;

	if (!plugin->getIcon().isNull())
	{
		iconPixmap = plugin->getIcon().pixmap(iconSize);
	}

	switch (plugin->getType())
	{
	case CC_STD_PLUGIN:
	{
		if (iconPixmap.isNull())
		{
			iconPixmap = QPixmap(":/CC/pluginManager/images/std_plugin.png").scaled(iconSize);
		}

		m_ui->mPluginTypeLabel->clear();
		break;
	}

	case CC_GL_FILTER_PLUGIN:
	{
		if (iconPixmap.isNull())
		{
			iconPixmap = QPixmap(":/CC/pluginManager/images/gl_plugin.png").scaled(iconSize);
		}

		m_ui->mPluginTypeLabel->setText(tr("GL Shader"));
		break;
	}

	case CC_IO_FILTER_PLUGIN:
	{
		if (iconPixmap.isNull())
		{
			iconPixmap = QPixmap(":/CC/pluginManager/images/io_plugin.png").scaled(iconSize);
		}

		m_ui->mPluginTypeLabel->setText(tr("I/O"));
		break;
	}
	}

	m_ui->mIcon->setPixmap(iconPixmap);

	m_ui->mNameLabel->setText(plugin->getName());
	m_ui->mDescriptionTextEdit->setHtml(plugin->getDescription());

	const QString referenceText = sFormatReferenceList(plugin->getReferences());

	if (!referenceText.isEmpty())
	{
		m_ui->mReferencesTextBrowser->setHtml(referenceText);
		m_ui->mReferencesLabel->show();
		m_ui->mReferencesTextBrowser->show();
	}
	else
	{
		m_ui->mReferencesLabel->hide();
		m_ui->mReferencesTextBrowser->hide();
		m_ui->mReferencesTextBrowser->clear();
	}

	const QString authorsText = sFormatContactList(plugin->getAuthors(), plugin->getName());

	if (!authorsText.isEmpty())
	{
		m_ui->mAuthorsTextBrowser->setHtml(authorsText);
	}
	else
	{
		m_ui->mAuthorsTextBrowser->clear();
	}

	const QString maintainersText = sFormatContactList(plugin->getMaintainers(), plugin->getName());

	if (!maintainersText.isEmpty())
	{
		m_ui->mMaintainerTextBrowser->setHtml(maintainersText);
	}
	else
	{
		m_ui->mMaintainerTextBrowser->clear();
	}
}
