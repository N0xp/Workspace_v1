/****************************************************************************
** Meta object code from reading C++ file 'PlotCurve.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.11.3)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../../../../src/rqt_multiplot_plugin/include/rqt_multiplot/PlotCurve.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'PlotCurve.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.11.3. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
QT_WARNING_PUSH
QT_WARNING_DISABLE_DEPRECATED
struct qt_meta_stringdata_rqt_multiplot__PlotCurve_t {
    QByteArrayData data[15];
    char stringdata0[263];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_rqt_multiplot__PlotCurve_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_rqt_multiplot__PlotCurve_t qt_meta_stringdata_rqt_multiplot__PlotCurve = {
    {
QT_MOC_LITERAL(0, 0, 24), // "rqt_multiplot::PlotCurve"
QT_MOC_LITERAL(1, 25, 21), // "preferredScaleChanged"
QT_MOC_LITERAL(2, 47, 0), // ""
QT_MOC_LITERAL(3, 48, 17), // "BoundingRectangle"
QT_MOC_LITERAL(4, 66, 6), // "bounds"
QT_MOC_LITERAL(5, 73, 15), // "replotRequested"
QT_MOC_LITERAL(6, 89, 18), // "configTitleChanged"
QT_MOC_LITERAL(7, 108, 5), // "title"
QT_MOC_LITERAL(8, 114, 23), // "configAxisConfigChanged"
QT_MOC_LITERAL(9, 138, 36), // "configColorConfigCurrentColor..."
QT_MOC_LITERAL(10, 175, 5), // "color"
QT_MOC_LITERAL(11, 181, 24), // "configStyleConfigChanged"
QT_MOC_LITERAL(12, 206, 23), // "configDataConfigChanged"
QT_MOC_LITERAL(13, 230, 26), // "dataSequencerPointReceived"
QT_MOC_LITERAL(14, 257, 5) // "point"

    },
    "rqt_multiplot::PlotCurve\0preferredScaleChanged\0"
    "\0BoundingRectangle\0bounds\0replotRequested\0"
    "configTitleChanged\0title\0"
    "configAxisConfigChanged\0"
    "configColorConfigCurrentColorChanged\0"
    "color\0configStyleConfigChanged\0"
    "configDataConfigChanged\0"
    "dataSequencerPointReceived\0point"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_rqt_multiplot__PlotCurve[] = {

 // content:
       7,       // revision
       0,       // classname
       0,    0, // classinfo
       8,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       2,       // signalCount

 // signals: name, argc, parameters, tag, flags
       1,    1,   54,    2, 0x06 /* Public */,
       5,    0,   57,    2, 0x06 /* Public */,

 // slots: name, argc, parameters, tag, flags
       6,    1,   58,    2, 0x08 /* Private */,
       8,    0,   61,    2, 0x08 /* Private */,
       9,    1,   62,    2, 0x08 /* Private */,
      11,    0,   65,    2, 0x08 /* Private */,
      12,    0,   66,    2, 0x08 /* Private */,
      13,    1,   67,    2, 0x08 /* Private */,

 // signals: parameters
    QMetaType::Void, 0x80000000 | 3,    4,
    QMetaType::Void,

 // slots: parameters
    QMetaType::Void, QMetaType::QString,    7,
    QMetaType::Void,
    QMetaType::Void, QMetaType::QColor,   10,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void, QMetaType::QPointF,   14,

       0        // eod
};

void rqt_multiplot::PlotCurve::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        PlotCurve *_t = static_cast<PlotCurve *>(_o);
        Q_UNUSED(_t)
        switch (_id) {
        case 0: _t->preferredScaleChanged((*reinterpret_cast< const BoundingRectangle(*)>(_a[1]))); break;
        case 1: _t->replotRequested(); break;
        case 2: _t->configTitleChanged((*reinterpret_cast< const QString(*)>(_a[1]))); break;
        case 3: _t->configAxisConfigChanged(); break;
        case 4: _t->configColorConfigCurrentColorChanged((*reinterpret_cast< const QColor(*)>(_a[1]))); break;
        case 5: _t->configStyleConfigChanged(); break;
        case 6: _t->configDataConfigChanged(); break;
        case 7: _t->dataSequencerPointReceived((*reinterpret_cast< const QPointF(*)>(_a[1]))); break;
        default: ;
        }
    } else if (_c == QMetaObject::IndexOfMethod) {
        int *result = reinterpret_cast<int *>(_a[0]);
        {
            using _t = void (PlotCurve::*)(const BoundingRectangle & );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&PlotCurve::preferredScaleChanged)) {
                *result = 0;
                return;
            }
        }
        {
            using _t = void (PlotCurve::*)();
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&PlotCurve::replotRequested)) {
                *result = 1;
                return;
            }
        }
    }
}

QT_INIT_METAOBJECT const QMetaObject rqt_multiplot::PlotCurve::staticMetaObject = {
    { &QObject::staticMetaObject, qt_meta_stringdata_rqt_multiplot__PlotCurve.data,
      qt_meta_data_rqt_multiplot__PlotCurve,  qt_static_metacall, nullptr, nullptr}
};


const QMetaObject *rqt_multiplot::PlotCurve::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *rqt_multiplot::PlotCurve::qt_metacast(const char *_clname)
{
    if (!_clname) return nullptr;
    if (!strcmp(_clname, qt_meta_stringdata_rqt_multiplot__PlotCurve.stringdata0))
        return static_cast<void*>(this);
    if (!strcmp(_clname, "QwtPlotCurve"))
        return static_cast< QwtPlotCurve*>(this);
    return QObject::qt_metacast(_clname);
}

int rqt_multiplot::PlotCurve::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QObject::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 8)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 8;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 8)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 8;
    }
    return _id;
}

// SIGNAL 0
void rqt_multiplot::PlotCurve::preferredScaleChanged(const BoundingRectangle & _t1)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 0, _a);
}

// SIGNAL 1
void rqt_multiplot::PlotCurve::replotRequested()
{
    QMetaObject::activate(this, &staticMetaObject, 1, nullptr);
}
QT_WARNING_POP
QT_END_MOC_NAMESPACE
