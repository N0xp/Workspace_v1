/****************************************************************************
** Meta object code from reading C++ file 'PlotAxisConfigWidget.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.11.3)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../../../../src/rqt_multiplot_plugin/include/rqt_multiplot/PlotAxisConfigWidget.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'PlotAxisConfigWidget.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.11.3. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
QT_WARNING_PUSH
QT_WARNING_DISABLE_DEPRECATED
struct qt_meta_stringdata_rqt_multiplot__PlotAxisConfigWidget_t {
    QByteArrayData data[12];
    char stringdata0[228];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_rqt_multiplot__PlotAxisConfigWidget_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_rqt_multiplot__PlotAxisConfigWidget_t qt_meta_stringdata_rqt_multiplot__PlotAxisConfigWidget = {
    {
QT_MOC_LITERAL(0, 0, 35), // "rqt_multiplot::PlotAxisConfig..."
QT_MOC_LITERAL(1, 36, 22), // "configTitleTypeChanged"
QT_MOC_LITERAL(2, 59, 0), // ""
QT_MOC_LITERAL(3, 60, 4), // "type"
QT_MOC_LITERAL(4, 65, 24), // "configCustomTitleChanged"
QT_MOC_LITERAL(5, 90, 5), // "title"
QT_MOC_LITERAL(6, 96, 25), // "configTitleVisibleChanged"
QT_MOC_LITERAL(7, 122, 7), // "visible"
QT_MOC_LITERAL(8, 130, 29), // "checkBoxTitleAutoStateChanged"
QT_MOC_LITERAL(9, 160, 5), // "state"
QT_MOC_LITERAL(10, 166, 28), // "lineEditTitleEditingFinished"
QT_MOC_LITERAL(11, 195, 32) // "checkBoxTitleVisibleStateChanged"

    },
    "rqt_multiplot::PlotAxisConfigWidget\0"
    "configTitleTypeChanged\0\0type\0"
    "configCustomTitleChanged\0title\0"
    "configTitleVisibleChanged\0visible\0"
    "checkBoxTitleAutoStateChanged\0state\0"
    "lineEditTitleEditingFinished\0"
    "checkBoxTitleVisibleStateChanged"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_rqt_multiplot__PlotAxisConfigWidget[] = {

 // content:
       7,       // revision
       0,       // classname
       0,    0, // classinfo
       6,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: name, argc, parameters, tag, flags
       1,    1,   44,    2, 0x08 /* Private */,
       4,    1,   47,    2, 0x08 /* Private */,
       6,    1,   50,    2, 0x08 /* Private */,
       8,    1,   53,    2, 0x08 /* Private */,
      10,    0,   56,    2, 0x08 /* Private */,
      11,    1,   57,    2, 0x08 /* Private */,

 // slots: parameters
    QMetaType::Void, QMetaType::Int,    3,
    QMetaType::Void, QMetaType::QString,    5,
    QMetaType::Void, QMetaType::Bool,    7,
    QMetaType::Void, QMetaType::Int,    9,
    QMetaType::Void,
    QMetaType::Void, QMetaType::Int,    9,

       0        // eod
};

void rqt_multiplot::PlotAxisConfigWidget::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        PlotAxisConfigWidget *_t = static_cast<PlotAxisConfigWidget *>(_o);
        Q_UNUSED(_t)
        switch (_id) {
        case 0: _t->configTitleTypeChanged((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 1: _t->configCustomTitleChanged((*reinterpret_cast< const QString(*)>(_a[1]))); break;
        case 2: _t->configTitleVisibleChanged((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 3: _t->checkBoxTitleAutoStateChanged((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 4: _t->lineEditTitleEditingFinished(); break;
        case 5: _t->checkBoxTitleVisibleStateChanged((*reinterpret_cast< int(*)>(_a[1]))); break;
        default: ;
        }
    }
}

QT_INIT_METAOBJECT const QMetaObject rqt_multiplot::PlotAxisConfigWidget::staticMetaObject = {
    { &QWidget::staticMetaObject, qt_meta_stringdata_rqt_multiplot__PlotAxisConfigWidget.data,
      qt_meta_data_rqt_multiplot__PlotAxisConfigWidget,  qt_static_metacall, nullptr, nullptr}
};


const QMetaObject *rqt_multiplot::PlotAxisConfigWidget::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *rqt_multiplot::PlotAxisConfigWidget::qt_metacast(const char *_clname)
{
    if (!_clname) return nullptr;
    if (!strcmp(_clname, qt_meta_stringdata_rqt_multiplot__PlotAxisConfigWidget.stringdata0))
        return static_cast<void*>(this);
    return QWidget::qt_metacast(_clname);
}

int rqt_multiplot::PlotAxisConfigWidget::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QWidget::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 6)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 6;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 6)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 6;
    }
    return _id;
}
QT_WARNING_POP
QT_END_MOC_NAMESPACE
