/****************************************************************************
** Meta object code from reading C++ file 'PlotCursor.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.11.3)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../../../../src/rqt_multiplot_plugin/include/rqt_multiplot/PlotCursor.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'PlotCursor.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.11.3. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
QT_WARNING_PUSH
QT_WARNING_DISABLE_DEPRECATED
struct qt_meta_stringdata_rqt_multiplot__PlotCursor_t {
    QByteArrayData data[8];
    char stringdata0[130];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_rqt_multiplot__PlotCursor_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_rqt_multiplot__PlotCursor_t qt_meta_stringdata_rqt_multiplot__PlotCursor = {
    {
QT_MOC_LITERAL(0, 0, 25), // "rqt_multiplot::PlotCursor"
QT_MOC_LITERAL(1, 26, 13), // "activeChanged"
QT_MOC_LITERAL(2, 40, 0), // ""
QT_MOC_LITERAL(3, 41, 6), // "active"
QT_MOC_LITERAL(4, 48, 22), // "currentPositionChanged"
QT_MOC_LITERAL(5, 71, 8), // "position"
QT_MOC_LITERAL(6, 80, 24), // "plotXAxisScaleDivChanged"
QT_MOC_LITERAL(7, 105, 24) // "plotYAxisScaleDivChanged"

    },
    "rqt_multiplot::PlotCursor\0activeChanged\0"
    "\0active\0currentPositionChanged\0position\0"
    "plotXAxisScaleDivChanged\0"
    "plotYAxisScaleDivChanged"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_rqt_multiplot__PlotCursor[] = {

 // content:
       7,       // revision
       0,       // classname
       0,    0, // classinfo
       4,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       2,       // signalCount

 // signals: name, argc, parameters, tag, flags
       1,    1,   34,    2, 0x06 /* Public */,
       4,    1,   37,    2, 0x06 /* Public */,

 // slots: name, argc, parameters, tag, flags
       6,    0,   40,    2, 0x08 /* Private */,
       7,    0,   41,    2, 0x08 /* Private */,

 // signals: parameters
    QMetaType::Void, QMetaType::Bool,    3,
    QMetaType::Void, QMetaType::QPointF,    5,

 // slots: parameters
    QMetaType::Void,
    QMetaType::Void,

       0        // eod
};

void rqt_multiplot::PlotCursor::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        PlotCursor *_t = static_cast<PlotCursor *>(_o);
        Q_UNUSED(_t)
        switch (_id) {
        case 0: _t->activeChanged((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 1: _t->currentPositionChanged((*reinterpret_cast< const QPointF(*)>(_a[1]))); break;
        case 2: _t->plotXAxisScaleDivChanged(); break;
        case 3: _t->plotYAxisScaleDivChanged(); break;
        default: ;
        }
    } else if (_c == QMetaObject::IndexOfMethod) {
        int *result = reinterpret_cast<int *>(_a[0]);
        {
            using _t = void (PlotCursor::*)(bool );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&PlotCursor::activeChanged)) {
                *result = 0;
                return;
            }
        }
        {
            using _t = void (PlotCursor::*)(const QPointF & );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&PlotCursor::currentPositionChanged)) {
                *result = 1;
                return;
            }
        }
    }
}

QT_INIT_METAOBJECT const QMetaObject rqt_multiplot::PlotCursor::staticMetaObject = {
    { &QwtPlotPicker::staticMetaObject, qt_meta_stringdata_rqt_multiplot__PlotCursor.data,
      qt_meta_data_rqt_multiplot__PlotCursor,  qt_static_metacall, nullptr, nullptr}
};


const QMetaObject *rqt_multiplot::PlotCursor::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *rqt_multiplot::PlotCursor::qt_metacast(const char *_clname)
{
    if (!_clname) return nullptr;
    if (!strcmp(_clname, qt_meta_stringdata_rqt_multiplot__PlotCursor.stringdata0))
        return static_cast<void*>(this);
    return QwtPlotPicker::qt_metacast(_clname);
}

int rqt_multiplot::PlotCursor::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QwtPlotPicker::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 4)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 4;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 4)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 4;
    }
    return _id;
}

// SIGNAL 0
void rqt_multiplot::PlotCursor::activeChanged(bool _t1)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 0, _a);
}

// SIGNAL 1
void rqt_multiplot::PlotCursor::currentPositionChanged(const QPointF & _t1)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 1, _a);
}
QT_WARNING_POP
QT_END_MOC_NAMESPACE
