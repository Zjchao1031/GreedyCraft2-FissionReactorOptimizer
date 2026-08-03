#pragma once

#include "Grid.h"

#include <QColor>
#include <QTableWidget>

class QMouseEvent;
class QWheelEvent;

class ReactorGridWidget : public QTableWidget {
    Q_OBJECT

public:
    explicit ReactorGridWidget(QWidget* parent = nullptr);

    void setGrid(const ncfr::Grid* grid);
    void setLayer(int layer);

    static QColor colorForKind(ncfr::BlockKind kind);

signals:
    void fuelCellClicked(int x, int y, int z, int index);

private:
    void mousePressEvent(QMouseEvent* event) override;
    void mouseMoveEvent(QMouseEvent* event) override;
    void mouseReleaseEvent(QMouseEvent* event) override;
    void wheelEvent(QWheelEvent* event) override;

    void refresh();

    static QColor colorForBlock(const ncfr::Block& block);
    static QString shortNameForBlock(const ncfr::Block& block);

    const ncfr::Grid* grid_ = nullptr;
    int layer_ = 0;
};
