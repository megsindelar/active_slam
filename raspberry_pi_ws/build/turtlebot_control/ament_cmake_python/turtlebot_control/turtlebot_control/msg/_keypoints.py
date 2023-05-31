# generated from rosidl_generator_py/resource/_idl.py.em
# with input from turtlebot_control:msg/Keypoints.idl
# generated code does not contain a copyright notice


# Import statements for member types

# Member 'x_1'
# Member 'y_1'
# Member 'size_1'
# Member 'angle_1'
# Member 'response_1'
# Member 'octave_1'
# Member 'class_id_1'
# Member 'x_2'
# Member 'y_2'
# Member 'size_2'
# Member 'angle_2'
# Member 'response_2'
# Member 'octave_2'
# Member 'class_id_2'
import array  # noqa: E402, I100

import builtins  # noqa: E402, I100

import math  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_Keypoints(type):
    """Metaclass of message 'Keypoints'."""

    _CREATE_ROS_MESSAGE = None
    _CONVERT_FROM_PY = None
    _CONVERT_TO_PY = None
    _DESTROY_ROS_MESSAGE = None
    _TYPE_SUPPORT = None

    __constants = {
    }

    @classmethod
    def __import_type_support__(cls):
        try:
            from rosidl_generator_py import import_type_support
            module = import_type_support('turtlebot_control')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'turtlebot_control.msg.Keypoints')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__keypoints
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__keypoints
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__keypoints
            cls._TYPE_SUPPORT = module.type_support_msg__msg__keypoints
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__keypoints

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class Keypoints(metaclass=Metaclass_Keypoints):
    """Message class 'Keypoints'."""

    __slots__ = [
        '_x_1',
        '_y_1',
        '_size_1',
        '_angle_1',
        '_response_1',
        '_octave_1',
        '_class_id_1',
        '_x_2',
        '_y_2',
        '_size_2',
        '_angle_2',
        '_response_2',
        '_octave_2',
        '_class_id_2',
    ]

    _fields_and_field_types = {
        'x_1': 'sequence<double>',
        'y_1': 'sequence<double>',
        'size_1': 'sequence<double>',
        'angle_1': 'sequence<double>',
        'response_1': 'sequence<double>',
        'octave_1': 'sequence<int32>',
        'class_id_1': 'sequence<int32>',
        'x_2': 'sequence<double>',
        'y_2': 'sequence<double>',
        'size_2': 'sequence<double>',
        'angle_2': 'sequence<double>',
        'response_2': 'sequence<double>',
        'octave_2': 'sequence<int32>',
        'class_id_2': 'sequence<int32>',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.BasicType('double')),  # noqa: E501
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.BasicType('double')),  # noqa: E501
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.BasicType('double')),  # noqa: E501
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.BasicType('double')),  # noqa: E501
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.BasicType('double')),  # noqa: E501
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.BasicType('int32')),  # noqa: E501
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.BasicType('int32')),  # noqa: E501
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.BasicType('double')),  # noqa: E501
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.BasicType('double')),  # noqa: E501
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.BasicType('double')),  # noqa: E501
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.BasicType('double')),  # noqa: E501
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.BasicType('double')),  # noqa: E501
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.BasicType('int32')),  # noqa: E501
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.BasicType('int32')),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.x_1 = array.array('d', kwargs.get('x_1', []))
        self.y_1 = array.array('d', kwargs.get('y_1', []))
        self.size_1 = array.array('d', kwargs.get('size_1', []))
        self.angle_1 = array.array('d', kwargs.get('angle_1', []))
        self.response_1 = array.array('d', kwargs.get('response_1', []))
        self.octave_1 = array.array('i', kwargs.get('octave_1', []))
        self.class_id_1 = array.array('i', kwargs.get('class_id_1', []))
        self.x_2 = array.array('d', kwargs.get('x_2', []))
        self.y_2 = array.array('d', kwargs.get('y_2', []))
        self.size_2 = array.array('d', kwargs.get('size_2', []))
        self.angle_2 = array.array('d', kwargs.get('angle_2', []))
        self.response_2 = array.array('d', kwargs.get('response_2', []))
        self.octave_2 = array.array('i', kwargs.get('octave_2', []))
        self.class_id_2 = array.array('i', kwargs.get('class_id_2', []))

    def __repr__(self):
        typename = self.__class__.__module__.split('.')
        typename.pop()
        typename.append(self.__class__.__name__)
        args = []
        for s, t in zip(self.__slots__, self.SLOT_TYPES):
            field = getattr(self, s)
            fieldstr = repr(field)
            # We use Python array type for fields that can be directly stored
            # in them, and "normal" sequences for everything else.  If it is
            # a type that we store in an array, strip off the 'array' portion.
            if (
                isinstance(t, rosidl_parser.definition.AbstractSequence) and
                isinstance(t.value_type, rosidl_parser.definition.BasicType) and
                t.value_type.typename in ['float', 'double', 'int8', 'uint8', 'int16', 'uint16', 'int32', 'uint32', 'int64', 'uint64']
            ):
                if len(field) == 0:
                    fieldstr = '[]'
                else:
                    assert fieldstr.startswith('array(')
                    prefix = "array('X', "
                    suffix = ')'
                    fieldstr = fieldstr[len(prefix):-len(suffix)]
            args.append(s[1:] + '=' + fieldstr)
        return '%s(%s)' % ('.'.join(typename), ', '.join(args))

    def __eq__(self, other):
        if not isinstance(other, self.__class__):
            return False
        if self.x_1 != other.x_1:
            return False
        if self.y_1 != other.y_1:
            return False
        if self.size_1 != other.size_1:
            return False
        if self.angle_1 != other.angle_1:
            return False
        if self.response_1 != other.response_1:
            return False
        if self.octave_1 != other.octave_1:
            return False
        if self.class_id_1 != other.class_id_1:
            return False
        if self.x_2 != other.x_2:
            return False
        if self.y_2 != other.y_2:
            return False
        if self.size_2 != other.size_2:
            return False
        if self.angle_2 != other.angle_2:
            return False
        if self.response_2 != other.response_2:
            return False
        if self.octave_2 != other.octave_2:
            return False
        if self.class_id_2 != other.class_id_2:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def x_1(self):
        """Message field 'x_1'."""
        return self._x_1

    @x_1.setter
    def x_1(self, value):
        if isinstance(value, array.array):
            assert value.typecode == 'd', \
                "The 'x_1' array.array() must have the type code of 'd'"
            self._x_1 = value
            return
        if __debug__:
            from collections.abc import Sequence
            from collections.abc import Set
            from collections import UserList
            from collections import UserString
            assert \
                ((isinstance(value, Sequence) or
                  isinstance(value, Set) or
                  isinstance(value, UserList)) and
                 not isinstance(value, str) and
                 not isinstance(value, UserString) and
                 all(isinstance(v, float) for v in value) and
                 all(not (val < -1.7976931348623157e+308 or val > 1.7976931348623157e+308) or math.isinf(val) for val in value)), \
                "The 'x_1' field must be a set or sequence and each value of type 'float' and each double in [-179769313486231570814527423731704356798070567525844996598917476803157260780028538760589558632766878171540458953514382464234321326889464182768467546703537516986049910576551282076245490090389328944075868508455133942304583236903222948165808559332123348274797826204144723168738177180919299881250404026184124858368.000000, 179769313486231570814527423731704356798070567525844996598917476803157260780028538760589558632766878171540458953514382464234321326889464182768467546703537516986049910576551282076245490090389328944075868508455133942304583236903222948165808559332123348274797826204144723168738177180919299881250404026184124858368.000000]"
        self._x_1 = array.array('d', value)

    @builtins.property
    def y_1(self):
        """Message field 'y_1'."""
        return self._y_1

    @y_1.setter
    def y_1(self, value):
        if isinstance(value, array.array):
            assert value.typecode == 'd', \
                "The 'y_1' array.array() must have the type code of 'd'"
            self._y_1 = value
            return
        if __debug__:
            from collections.abc import Sequence
            from collections.abc import Set
            from collections import UserList
            from collections import UserString
            assert \
                ((isinstance(value, Sequence) or
                  isinstance(value, Set) or
                  isinstance(value, UserList)) and
                 not isinstance(value, str) and
                 not isinstance(value, UserString) and
                 all(isinstance(v, float) for v in value) and
                 all(not (val < -1.7976931348623157e+308 or val > 1.7976931348623157e+308) or math.isinf(val) for val in value)), \
                "The 'y_1' field must be a set or sequence and each value of type 'float' and each double in [-179769313486231570814527423731704356798070567525844996598917476803157260780028538760589558632766878171540458953514382464234321326889464182768467546703537516986049910576551282076245490090389328944075868508455133942304583236903222948165808559332123348274797826204144723168738177180919299881250404026184124858368.000000, 179769313486231570814527423731704356798070567525844996598917476803157260780028538760589558632766878171540458953514382464234321326889464182768467546703537516986049910576551282076245490090389328944075868508455133942304583236903222948165808559332123348274797826204144723168738177180919299881250404026184124858368.000000]"
        self._y_1 = array.array('d', value)

    @builtins.property
    def size_1(self):
        """Message field 'size_1'."""
        return self._size_1

    @size_1.setter
    def size_1(self, value):
        if isinstance(value, array.array):
            assert value.typecode == 'd', \
                "The 'size_1' array.array() must have the type code of 'd'"
            self._size_1 = value
            return
        if __debug__:
            from collections.abc import Sequence
            from collections.abc import Set
            from collections import UserList
            from collections import UserString
            assert \
                ((isinstance(value, Sequence) or
                  isinstance(value, Set) or
                  isinstance(value, UserList)) and
                 not isinstance(value, str) and
                 not isinstance(value, UserString) and
                 all(isinstance(v, float) for v in value) and
                 all(not (val < -1.7976931348623157e+308 or val > 1.7976931348623157e+308) or math.isinf(val) for val in value)), \
                "The 'size_1' field must be a set or sequence and each value of type 'float' and each double in [-179769313486231570814527423731704356798070567525844996598917476803157260780028538760589558632766878171540458953514382464234321326889464182768467546703537516986049910576551282076245490090389328944075868508455133942304583236903222948165808559332123348274797826204144723168738177180919299881250404026184124858368.000000, 179769313486231570814527423731704356798070567525844996598917476803157260780028538760589558632766878171540458953514382464234321326889464182768467546703537516986049910576551282076245490090389328944075868508455133942304583236903222948165808559332123348274797826204144723168738177180919299881250404026184124858368.000000]"
        self._size_1 = array.array('d', value)

    @builtins.property
    def angle_1(self):
        """Message field 'angle_1'."""
        return self._angle_1

    @angle_1.setter
    def angle_1(self, value):
        if isinstance(value, array.array):
            assert value.typecode == 'd', \
                "The 'angle_1' array.array() must have the type code of 'd'"
            self._angle_1 = value
            return
        if __debug__:
            from collections.abc import Sequence
            from collections.abc import Set
            from collections import UserList
            from collections import UserString
            assert \
                ((isinstance(value, Sequence) or
                  isinstance(value, Set) or
                  isinstance(value, UserList)) and
                 not isinstance(value, str) and
                 not isinstance(value, UserString) and
                 all(isinstance(v, float) for v in value) and
                 all(not (val < -1.7976931348623157e+308 or val > 1.7976931348623157e+308) or math.isinf(val) for val in value)), \
                "The 'angle_1' field must be a set or sequence and each value of type 'float' and each double in [-179769313486231570814527423731704356798070567525844996598917476803157260780028538760589558632766878171540458953514382464234321326889464182768467546703537516986049910576551282076245490090389328944075868508455133942304583236903222948165808559332123348274797826204144723168738177180919299881250404026184124858368.000000, 179769313486231570814527423731704356798070567525844996598917476803157260780028538760589558632766878171540458953514382464234321326889464182768467546703537516986049910576551282076245490090389328944075868508455133942304583236903222948165808559332123348274797826204144723168738177180919299881250404026184124858368.000000]"
        self._angle_1 = array.array('d', value)

    @builtins.property
    def response_1(self):
        """Message field 'response_1'."""
        return self._response_1

    @response_1.setter
    def response_1(self, value):
        if isinstance(value, array.array):
            assert value.typecode == 'd', \
                "The 'response_1' array.array() must have the type code of 'd'"
            self._response_1 = value
            return
        if __debug__:
            from collections.abc import Sequence
            from collections.abc import Set
            from collections import UserList
            from collections import UserString
            assert \
                ((isinstance(value, Sequence) or
                  isinstance(value, Set) or
                  isinstance(value, UserList)) and
                 not isinstance(value, str) and
                 not isinstance(value, UserString) and
                 all(isinstance(v, float) for v in value) and
                 all(not (val < -1.7976931348623157e+308 or val > 1.7976931348623157e+308) or math.isinf(val) for val in value)), \
                "The 'response_1' field must be a set or sequence and each value of type 'float' and each double in [-179769313486231570814527423731704356798070567525844996598917476803157260780028538760589558632766878171540458953514382464234321326889464182768467546703537516986049910576551282076245490090389328944075868508455133942304583236903222948165808559332123348274797826204144723168738177180919299881250404026184124858368.000000, 179769313486231570814527423731704356798070567525844996598917476803157260780028538760589558632766878171540458953514382464234321326889464182768467546703537516986049910576551282076245490090389328944075868508455133942304583236903222948165808559332123348274797826204144723168738177180919299881250404026184124858368.000000]"
        self._response_1 = array.array('d', value)

    @builtins.property
    def octave_1(self):
        """Message field 'octave_1'."""
        return self._octave_1

    @octave_1.setter
    def octave_1(self, value):
        if isinstance(value, array.array):
            assert value.typecode == 'i', \
                "The 'octave_1' array.array() must have the type code of 'i'"
            self._octave_1 = value
            return
        if __debug__:
            from collections.abc import Sequence
            from collections.abc import Set
            from collections import UserList
            from collections import UserString
            assert \
                ((isinstance(value, Sequence) or
                  isinstance(value, Set) or
                  isinstance(value, UserList)) and
                 not isinstance(value, str) and
                 not isinstance(value, UserString) and
                 all(isinstance(v, int) for v in value) and
                 all(val >= -2147483648 and val < 2147483648 for val in value)), \
                "The 'octave_1' field must be a set or sequence and each value of type 'int' and each integer in [-2147483648, 2147483647]"
        self._octave_1 = array.array('i', value)

    @builtins.property
    def class_id_1(self):
        """Message field 'class_id_1'."""
        return self._class_id_1

    @class_id_1.setter
    def class_id_1(self, value):
        if isinstance(value, array.array):
            assert value.typecode == 'i', \
                "The 'class_id_1' array.array() must have the type code of 'i'"
            self._class_id_1 = value
            return
        if __debug__:
            from collections.abc import Sequence
            from collections.abc import Set
            from collections import UserList
            from collections import UserString
            assert \
                ((isinstance(value, Sequence) or
                  isinstance(value, Set) or
                  isinstance(value, UserList)) and
                 not isinstance(value, str) and
                 not isinstance(value, UserString) and
                 all(isinstance(v, int) for v in value) and
                 all(val >= -2147483648 and val < 2147483648 for val in value)), \
                "The 'class_id_1' field must be a set or sequence and each value of type 'int' and each integer in [-2147483648, 2147483647]"
        self._class_id_1 = array.array('i', value)

    @builtins.property
    def x_2(self):
        """Message field 'x_2'."""
        return self._x_2

    @x_2.setter
    def x_2(self, value):
        if isinstance(value, array.array):
            assert value.typecode == 'd', \
                "The 'x_2' array.array() must have the type code of 'd'"
            self._x_2 = value
            return
        if __debug__:
            from collections.abc import Sequence
            from collections.abc import Set
            from collections import UserList
            from collections import UserString
            assert \
                ((isinstance(value, Sequence) or
                  isinstance(value, Set) or
                  isinstance(value, UserList)) and
                 not isinstance(value, str) and
                 not isinstance(value, UserString) and
                 all(isinstance(v, float) for v in value) and
                 all(not (val < -1.7976931348623157e+308 or val > 1.7976931348623157e+308) or math.isinf(val) for val in value)), \
                "The 'x_2' field must be a set or sequence and each value of type 'float' and each double in [-179769313486231570814527423731704356798070567525844996598917476803157260780028538760589558632766878171540458953514382464234321326889464182768467546703537516986049910576551282076245490090389328944075868508455133942304583236903222948165808559332123348274797826204144723168738177180919299881250404026184124858368.000000, 179769313486231570814527423731704356798070567525844996598917476803157260780028538760589558632766878171540458953514382464234321326889464182768467546703537516986049910576551282076245490090389328944075868508455133942304583236903222948165808559332123348274797826204144723168738177180919299881250404026184124858368.000000]"
        self._x_2 = array.array('d', value)

    @builtins.property
    def y_2(self):
        """Message field 'y_2'."""
        return self._y_2

    @y_2.setter
    def y_2(self, value):
        if isinstance(value, array.array):
            assert value.typecode == 'd', \
                "The 'y_2' array.array() must have the type code of 'd'"
            self._y_2 = value
            return
        if __debug__:
            from collections.abc import Sequence
            from collections.abc import Set
            from collections import UserList
            from collections import UserString
            assert \
                ((isinstance(value, Sequence) or
                  isinstance(value, Set) or
                  isinstance(value, UserList)) and
                 not isinstance(value, str) and
                 not isinstance(value, UserString) and
                 all(isinstance(v, float) for v in value) and
                 all(not (val < -1.7976931348623157e+308 or val > 1.7976931348623157e+308) or math.isinf(val) for val in value)), \
                "The 'y_2' field must be a set or sequence and each value of type 'float' and each double in [-179769313486231570814527423731704356798070567525844996598917476803157260780028538760589558632766878171540458953514382464234321326889464182768467546703537516986049910576551282076245490090389328944075868508455133942304583236903222948165808559332123348274797826204144723168738177180919299881250404026184124858368.000000, 179769313486231570814527423731704356798070567525844996598917476803157260780028538760589558632766878171540458953514382464234321326889464182768467546703537516986049910576551282076245490090389328944075868508455133942304583236903222948165808559332123348274797826204144723168738177180919299881250404026184124858368.000000]"
        self._y_2 = array.array('d', value)

    @builtins.property
    def size_2(self):
        """Message field 'size_2'."""
        return self._size_2

    @size_2.setter
    def size_2(self, value):
        if isinstance(value, array.array):
            assert value.typecode == 'd', \
                "The 'size_2' array.array() must have the type code of 'd'"
            self._size_2 = value
            return
        if __debug__:
            from collections.abc import Sequence
            from collections.abc import Set
            from collections import UserList
            from collections import UserString
            assert \
                ((isinstance(value, Sequence) or
                  isinstance(value, Set) or
                  isinstance(value, UserList)) and
                 not isinstance(value, str) and
                 not isinstance(value, UserString) and
                 all(isinstance(v, float) for v in value) and
                 all(not (val < -1.7976931348623157e+308 or val > 1.7976931348623157e+308) or math.isinf(val) for val in value)), \
                "The 'size_2' field must be a set or sequence and each value of type 'float' and each double in [-179769313486231570814527423731704356798070567525844996598917476803157260780028538760589558632766878171540458953514382464234321326889464182768467546703537516986049910576551282076245490090389328944075868508455133942304583236903222948165808559332123348274797826204144723168738177180919299881250404026184124858368.000000, 179769313486231570814527423731704356798070567525844996598917476803157260780028538760589558632766878171540458953514382464234321326889464182768467546703537516986049910576551282076245490090389328944075868508455133942304583236903222948165808559332123348274797826204144723168738177180919299881250404026184124858368.000000]"
        self._size_2 = array.array('d', value)

    @builtins.property
    def angle_2(self):
        """Message field 'angle_2'."""
        return self._angle_2

    @angle_2.setter
    def angle_2(self, value):
        if isinstance(value, array.array):
            assert value.typecode == 'd', \
                "The 'angle_2' array.array() must have the type code of 'd'"
            self._angle_2 = value
            return
        if __debug__:
            from collections.abc import Sequence
            from collections.abc import Set
            from collections import UserList
            from collections import UserString
            assert \
                ((isinstance(value, Sequence) or
                  isinstance(value, Set) or
                  isinstance(value, UserList)) and
                 not isinstance(value, str) and
                 not isinstance(value, UserString) and
                 all(isinstance(v, float) for v in value) and
                 all(not (val < -1.7976931348623157e+308 or val > 1.7976931348623157e+308) or math.isinf(val) for val in value)), \
                "The 'angle_2' field must be a set or sequence and each value of type 'float' and each double in [-179769313486231570814527423731704356798070567525844996598917476803157260780028538760589558632766878171540458953514382464234321326889464182768467546703537516986049910576551282076245490090389328944075868508455133942304583236903222948165808559332123348274797826204144723168738177180919299881250404026184124858368.000000, 179769313486231570814527423731704356798070567525844996598917476803157260780028538760589558632766878171540458953514382464234321326889464182768467546703537516986049910576551282076245490090389328944075868508455133942304583236903222948165808559332123348274797826204144723168738177180919299881250404026184124858368.000000]"
        self._angle_2 = array.array('d', value)

    @builtins.property
    def response_2(self):
        """Message field 'response_2'."""
        return self._response_2

    @response_2.setter
    def response_2(self, value):
        if isinstance(value, array.array):
            assert value.typecode == 'd', \
                "The 'response_2' array.array() must have the type code of 'd'"
            self._response_2 = value
            return
        if __debug__:
            from collections.abc import Sequence
            from collections.abc import Set
            from collections import UserList
            from collections import UserString
            assert \
                ((isinstance(value, Sequence) or
                  isinstance(value, Set) or
                  isinstance(value, UserList)) and
                 not isinstance(value, str) and
                 not isinstance(value, UserString) and
                 all(isinstance(v, float) for v in value) and
                 all(not (val < -1.7976931348623157e+308 or val > 1.7976931348623157e+308) or math.isinf(val) for val in value)), \
                "The 'response_2' field must be a set or sequence and each value of type 'float' and each double in [-179769313486231570814527423731704356798070567525844996598917476803157260780028538760589558632766878171540458953514382464234321326889464182768467546703537516986049910576551282076245490090389328944075868508455133942304583236903222948165808559332123348274797826204144723168738177180919299881250404026184124858368.000000, 179769313486231570814527423731704356798070567525844996598917476803157260780028538760589558632766878171540458953514382464234321326889464182768467546703537516986049910576551282076245490090389328944075868508455133942304583236903222948165808559332123348274797826204144723168738177180919299881250404026184124858368.000000]"
        self._response_2 = array.array('d', value)

    @builtins.property
    def octave_2(self):
        """Message field 'octave_2'."""
        return self._octave_2

    @octave_2.setter
    def octave_2(self, value):
        if isinstance(value, array.array):
            assert value.typecode == 'i', \
                "The 'octave_2' array.array() must have the type code of 'i'"
            self._octave_2 = value
            return
        if __debug__:
            from collections.abc import Sequence
            from collections.abc import Set
            from collections import UserList
            from collections import UserString
            assert \
                ((isinstance(value, Sequence) or
                  isinstance(value, Set) or
                  isinstance(value, UserList)) and
                 not isinstance(value, str) and
                 not isinstance(value, UserString) and
                 all(isinstance(v, int) for v in value) and
                 all(val >= -2147483648 and val < 2147483648 for val in value)), \
                "The 'octave_2' field must be a set or sequence and each value of type 'int' and each integer in [-2147483648, 2147483647]"
        self._octave_2 = array.array('i', value)

    @builtins.property
    def class_id_2(self):
        """Message field 'class_id_2'."""
        return self._class_id_2

    @class_id_2.setter
    def class_id_2(self, value):
        if isinstance(value, array.array):
            assert value.typecode == 'i', \
                "The 'class_id_2' array.array() must have the type code of 'i'"
            self._class_id_2 = value
            return
        if __debug__:
            from collections.abc import Sequence
            from collections.abc import Set
            from collections import UserList
            from collections import UserString
            assert \
                ((isinstance(value, Sequence) or
                  isinstance(value, Set) or
                  isinstance(value, UserList)) and
                 not isinstance(value, str) and
                 not isinstance(value, UserString) and
                 all(isinstance(v, int) for v in value) and
                 all(val >= -2147483648 and val < 2147483648 for val in value)), \
                "The 'class_id_2' field must be a set or sequence and each value of type 'int' and each integer in [-2147483648, 2147483647]"
        self._class_id_2 = array.array('i', value)
