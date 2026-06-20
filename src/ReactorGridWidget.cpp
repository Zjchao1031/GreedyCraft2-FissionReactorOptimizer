#include "ReactorGridWidget.h"

#include <QBrush>
#include <QHeaderView>
#include <QModelIndex>
#include <QMouseEvent>
#include <QPainter>
#include <QStringList>
#include <QStyle>
#include <QStyledItemDelegate>
#include <QStyleOptionViewItem>
#include <QTableWidgetItem>
#include <QWheelEvent>

namespace {

bool isDark(const QColor& color) {
    return (0.299 * color.red() + 0.587 * color.green() + 0.114 * color.blue()) < 145.0;
}

QString coordinateLabel(int value, QChar axis) {
    return QString("%1=%2").arg(axis).arg(value);
}

class StableBackgroundDelegate : public QStyledItemDelegate {
public:
    using QStyledItemDelegate::QStyledItemDelegate;

    void paint(QPainter* painter, const QStyleOptionViewItem& option, const QModelIndex& index) const override {
        QStyleOptionViewItem stableOption(option);
        stableOption.state &= ~(QStyle::State_Selected | QStyle::State_HasFocus | QStyle::State_MouseOver);
        QStyledItemDelegate::paint(painter, stableOption, index);
    }
};

} // namespace

ReactorGridWidget::ReactorGridWidget(QWidget* parent) : QTableWidget(parent) {
    setEditTriggers(QAbstractItemView::NoEditTriggers);
    setSelectionMode(QAbstractItemView::NoSelection);
    setFocusPolicy(Qt::NoFocus);
    setDragEnabled(false);
    setDragDropMode(QAbstractItemView::NoDragDrop);
    setAlternatingRowColors(false);
    setShowGrid(true);
    setWordWrap(true);
    setItemDelegate(new StableBackgroundDelegate(this));

    horizontalHeader()->setSectionResizeMode(QHeaderView::Fixed);
    horizontalHeader()->setDefaultSectionSize(112);
    verticalHeader()->setSectionResizeMode(QHeaderView::Fixed);
    verticalHeader()->setDefaultSectionSize(64);
    verticalHeader()->setDefaultAlignment(Qt::AlignCenter);

    setMinimumSize(560, 420);
}

void ReactorGridWidget::setGrid(const ncfr::Grid* grid) {
    grid_ = grid;
    layer_ = 0;
    refresh();
}

void ReactorGridWidget::setLayer(int layer) {
    layer_ = layer;
    refresh();
}

void ReactorGridWidget::mousePressEvent(QMouseEvent* event) {
    clearSelection();
    setCurrentIndex(QModelIndex());
    event->accept();
}

void ReactorGridWidget::mouseMoveEvent(QMouseEvent* event) {
    clearSelection();
    setCurrentIndex(QModelIndex());
    event->accept();
}

void ReactorGridWidget::mouseReleaseEvent(QMouseEvent* event) {
    clearSelection();
    setCurrentIndex(QModelIndex());
    if (event->button() == Qt::LeftButton && grid_ != nullptr &&
        layer_ >= 0 && layer_ < grid_->depth()) {
        const QModelIndex clicked = indexAt(event->pos());
        if (clicked.isValid()) {
            const int x = clicked.column();
            const int y = clicked.row();
            if (grid_->inBounds(x, y, layer_)) {
                const ncfr::Block& block = grid_->at(x, y, layer_);
                if (block.kind == ncfr::BlockKind::FuelCell) {
                    emit fuelCellClicked(x, y, layer_, grid_->index(x, y, layer_));
                }
            }
        }
    }
    event->accept();
}

void ReactorGridWidget::wheelEvent(QWheelEvent* event) {
    QTableWidget::wheelEvent(event);
    clearSelection();
    setCurrentIndex(QModelIndex());
    viewport()->update();
}

QColor ReactorGridWidget::colorForKind(ncfr::BlockKind kind) {
    switch (kind) {
    case ncfr::BlockKind::Empty:
        return QColor("#F4F6F8");
    case ncfr::BlockKind::Casing:
        return QColor("#6B7280");
    case ncfr::BlockKind::Controller:
        return QColor("#4F46E5");
    case ncfr::BlockKind::CellPort:
        return QColor("#0284C7");
    case ncfr::BlockKind::IrradiatorPort:
        return QColor("#DB2777");
    case ncfr::BlockKind::VentIn:
    case ncfr::BlockKind::VentOut:
        return QColor("#10B981");
    case ncfr::BlockKind::Source:
        return QColor("#EAB308");
    case ncfr::BlockKind::FuelCell:
        return QColor("#DC2626");
    case ncfr::BlockKind::Moderator:
        return QColor("#0D9488");
    case ncfr::BlockKind::Reflector:
        return QColor("#F97316");
    case ncfr::BlockKind::Shield:
        return QColor("#7C3AED");
    case ncfr::BlockKind::Irradiator:
        return QColor("#BE185D");
    case ncfr::BlockKind::Sink:
        return QColor("#2563EB");
    }
    return QColor("#F4F6F8");
}

void ReactorGridWidget::refresh() {
    clear();

    if (grid_ == nullptr || layer_ < 0 || layer_ >= grid_->depth()) {
        setRowCount(0);
        setColumnCount(0);
        return;
    }

    setRowCount(grid_->height());
    setColumnCount(grid_->width());

    QStringList horizontalLabels;
    horizontalLabels.reserve(grid_->width());
    for (int x = 0; x < grid_->width(); ++x) {
        horizontalLabels << coordinateLabel(x, QChar('x'));
    }
    setHorizontalHeaderLabels(horizontalLabels);

    QStringList verticalLabels;
    verticalLabels.reserve(grid_->height());
    for (int y = 0; y < grid_->height(); ++y) {
        verticalLabels << coordinateLabel(y, QChar('z'));
    }
    setVerticalHeaderLabels(verticalLabels);

    for (int y = 0; y < grid_->height(); ++y) {
        for (int x = 0; x < grid_->width(); ++x) {
            const ncfr::Block& block = grid_->at(x, y, layer_);
            const QColor background = colorForBlock(block);
            auto* item = new QTableWidgetItem(shortNameForBlock(block));
            item->setFlags(Qt::ItemIsEnabled);
            item->setTextAlignment(Qt::AlignCenter);
            item->setBackground(QBrush(background));
            item->setForeground(QBrush(isDark(background) ? Qt::white : Qt::black));
            item->setToolTip(QString::fromUtf8("坐标: x=%1, y=%2, z=%3\n方块: %4")
                                 .arg(x)
                                 .arg(layer_ + 1)
                                 .arg(y)
                                 .arg(QString::fromStdString(ncfr::blockDisplayName(block))));
            setItem(y, x, item);
        }
    }
}

QColor ReactorGridWidget::colorForBlock(const ncfr::Block& block) {
    return colorForKind(block.kind);
}

QString ReactorGridWidget::shortNameForBlock(const ncfr::Block& block) {
    return QString::fromStdString(ncfr::blockDisplayName(block));
}
