/****************************************************************************
** Meta object code from reading C++ file 'UrlScheme.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.11.3)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../../../../src/rqt_multiplot_plugin/include/rqt_multiplot/UrlScheme.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'UrlScheme.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.11.3. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
QT_WARNING_PUSH
QT_WARNING_DISABLE_DEPRECATED
struct qt_meta_stringdata_rqt_multiplot__UrlScheme_t {
    QByteArrayData data[7];
    char stringdata0[74];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_rqt_multiplot__UrlScheme_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_rqt_multiplot__UrlScheme_t qt_meta_stringdata_rqt_multiplot__UrlScheme = {
    {
QT_MOC_LITERAL(0, 0, 24), // "rqt_multiplot::UrlScheme"
QT_MOC_LITERAL(1, 25, 12), // "resetStarted"
QT_MOC_LITERAL(2, 38, 0), // ""
QT_MOC_LITERAL(3, 39, 13), // "resetFinished"
QT_MOC_LITERAL(4, 53, 10), // "pathLoaded"
QT_MOC_LITERAL(5, 64, 4), // "host"
QT_MOC_LITERAL(6, 69, 4) // "path"

    },
    "rqt_multiplot::UrlScheme\0resetStarted\0"
    "\0resetFinished\0pathLoaded\0host\0path"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_rqt_multiplot__UrlScheme[] = {

 // content:
       7,       // revision
       0,       // classname
       0,    0, // classinfo
       3,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       3,       // signalCount

 // signals: name, argc, parameters, tag, flags
       1,    0,   29,    2, 0x06 /* Public */,
       3,    0,   30,    2, 0x06 /* Public */,
       4,    2,   31,    2, 0x06 /* Public */,

 // signals: parameters
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void, QMetaType::QString, QMetaType::QString,    5,    6,

       0        // eod
};

void rqt_multiplot::UrlScheme::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        UrlScheme *_t = static_cast<UrlScheme *>(_o);
        Q_UNUSED(_t)
        switch (_id) {
        case 0: _t->resetStarted(); break;
        case 1: _t->resetFinished(); break;
        case 2: _t->pathLoaded((*reinterpret_cast< const QString(*)>(_a[1])),(*reinterpret_cast< const QString(*)>(_a[2]))); break;
        default: ;
        }
    } else if (_c == QMetaObject::IndexOfMethod) {
        int *result = reinterpret_cast<int *>(_a[0]);
        {
            using _t = void (UrlScheme::*)();
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&UrlScheme::resetStarted)) {
                *result = 0;
                return;
            }
        }
        {
            using _t = void (UrlScheme::*)();
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&UrlScheme::resetFinished)) {
                *result = 1;
                return;
            }
        }
        {
            using _t = void (UrlScheme::*)(const QString & , const QString & );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&UrlScheme::pathLoaded)) {
                *result = 2;
                return;
            }
        }
    }
}

QT_INIT_METAOBJECT const QMetaObject rqt_multiplot::UrlScheme::staticMetaObject = {
    { &QObject::staticMetaObject, qt_meta_stringdata_rqt_multiplot__UrlScheme.data,
      qt_meta_data_rqt_multiplot__UrlScheme,  qt_static_metacall, nullptr, nullptr}
};


const QMetaObject *rqt_multiplot::UrlScheme::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *rqt_multiplot::UrlScheme::qt_metacast(const char *_clname)
{
    if (!_clname) return nullptr;
    if (!strcmp(_clname, qt_meta_stringdata_rqt_multiplot__UrlScheme.stringdata0))
        return static_cast<void*>(this);
    return QObject::qt_metacast(_clname);
}

int rqt_multiplot::UrlScheme::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QObject::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 3)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 3;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 3)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 3;
    }
    return _id;
}

// SIGNAL 0
void rqt_multiplot::UrlScheme::resetStarted()
{
    QMetaObject::activate(this, &staticMetaObject, 0, nullptr);
}

// SIGNAL 1
void rqt_multiplot::UrlScheme::resetFinished()
{
    QMetaObject::activate(this, &staticMetaObject, 1, nullptr);
}

// SIGNAL 2
void rqt_multiplot::UrlScheme::pathLoaded(const QString & _t1, const QString & _t2)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(&_t1)), const_cast<void*>(reinterpret_cast<const void*>(&_t2)) };
    QMetaObject::activate(this, &staticMetaObject, 2, _a);
}
QT_WARNING_POP
QT_END_MOC_NAMESPACE
