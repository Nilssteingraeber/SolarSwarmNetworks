"use strict";
var __assign = (this && this.__assign) || function () {
    __assign = Object.assign || function(t) {
        for (var s, i = 1, n = arguments.length; i < n; i++) {
            s = arguments[i];
            for (var p in s) if (Object.prototype.hasOwnProperty.call(s, p))
                t[p] = s[p];
        }
        return t;
    };
    return __assign.apply(this, arguments);
};
var __spreadArray = (this && this.__spreadArray) || function (to, from, pack) {
    if (pack || arguments.length === 2) for (var i = 0, l = from.length, ar; i < l; i++) {
        if (ar || !(i in from)) {
            if (!ar) ar = Array.prototype.slice.call(from, 0, i);
            ar[i] = from[i];
        }
    }
    return to.concat(ar || Array.prototype.slice.call(from));
};
var _a;
var _b, _c;
Object.defineProperty(exports, "__esModule", { value: true });
var vue_1 = require("vue");
var Service_1 = require("../../../models/Service");
// Import your custom components
var TextInput_vue_1 = require("./TextInput.vue");
// import FloatInput from './FloatInput.vue';
// import IntegerInput from './IntegerInput.vue';
var BooleanInput_vue_1 = require("./BooleanInput.vue");
var FloatInput_vue_1 = require("./FloatInput.vue");
var mdb_vue_ui_kit_1 = require("mdb-vue-ui-kit");
var oh_vue_icons_1 = require("oh-vue-icons");
var props = defineProps();
// Reactive object to hold input values
var inputs = (0, vue_1.reactive)({});
var currentTime = (0, vue_1.ref)('');
var isShrunk = (0, vue_1.ref)(false);
function toggleLoader() {
    isShrunk.value = !isShrunk.value;
}
function updateTime() {
    var now = new Date();
    var hours = String(now.getHours()).padStart(2, '0');
    var minutes = String(now.getMinutes()).padStart(2, '0');
    var seconds = String(now.getSeconds()).padStart(2, '0');
    currentTime.value = "".concat(hours, ":").concat(minutes, ":").concat(seconds);
}
(0, vue_1.onMounted)(function () {
    updateTime(); // set initial value
    var interval = setInterval(updateTime, 30000);
    (0, vue_1.onUnmounted)(function () {
        clearInterval(interval);
    });
});
// Initialize defaults
// props.service.parameters.forEach(param => {
//     inputs[param.name] = null;
// });
// Map InputDataType to components
var componentMap = (_a = {},
    _a[Service_1.DataType.Text] = TextInput_vue_1.default,
    _a[Service_1.DataType.Float] = FloatInput_vue_1.default,
    // [InputDataType.Integer]: IntegerInput,
    _a[Service_1.DataType.Boolean] = BooleanInput_vue_1.default,
    _a);
debugger; /* PartiallyEnd: #3632/scriptSetup.vue */
var __VLS_ctx = {};
var __VLS_components;
var __VLS_directives;
// CSS variable injection 
// CSS variable injection end 
var __VLS_0 = {}.MDBRow;
/** @type {[typeof __VLS_components.MDBRow, typeof __VLS_components.MDBRow, ]} */ ;
// @ts-ignore
var __VLS_1 = __VLS_asFunctionalComponent(__VLS_0, new __VLS_0(__assign({ class: "g-3" })));
var __VLS_2 = __VLS_1.apply(void 0, __spreadArray([__assign({ class: "g-3" })], __VLS_functionalComponentArgsRest(__VLS_1), false));
var __VLS_4 = {};
__VLS_3.slots.default;
for (var _i = 0, _d = __VLS_getVForSourceType((props.services)); _i < _d.length; _i++) {
    var service = _d[_i][0];
    var __VLS_5 = {}.MDBCol;
    /** @type {[typeof __VLS_components.MDBCol, typeof __VLS_components.MDBCol, ]} */ ;
    // @ts-ignore
    var __VLS_6 = __VLS_asFunctionalComponent(__VLS_5, new __VLS_5(__assign({ key: (service.name) }, { class: "col-12 col-md-6" })));
    var __VLS_7 = __VLS_6.apply(void 0, __spreadArray([__assign({ key: (service.name) }, { class: "col-12 col-md-6" })], __VLS_functionalComponentArgsRest(__VLS_6), false));
    __VLS_8.slots.default;
    __VLS_asFunctionalElement(__VLS_intrinsicElements.div, __VLS_intrinsicElements.div)(__assign({ class: "input-layer p-2 h-100 d-flex flex-column" }));
    __VLS_asFunctionalElement(__VLS_intrinsicElements.div, __VLS_intrinsicElements.div)(__assign({ class: "d-flex justify-content-between align-items-center mb-2" }));
    __VLS_asFunctionalElement(__VLS_intrinsicElements.div, __VLS_intrinsicElements.div)(__assign({ class: "service-name shadowy" }));
    (service.name);
    __VLS_asFunctionalElement(__VLS_intrinsicElements.div, __VLS_intrinsicElements.div)(__assign({ class: "flex-grow-1 mx-2" }));
    __VLS_asFunctionalElement(__VLS_intrinsicElements.div, __VLS_intrinsicElements.div)(__assign({ class: "loader-bar" }, { class: ({ shrink: __VLS_ctx.isShrunk }) }));
    if (service.executable) {
        __VLS_asFunctionalElement(__VLS_intrinsicElements.div, __VLS_intrinsicElements.div)(__assign({ onClick: (__VLS_ctx.toggleLoader) }, { class: "d-flex align-items-center" }));
        if (!service.immediateExecute) {
            __VLS_asFunctionalElement(__VLS_intrinsicElements.div, __VLS_intrinsicElements.div)(__assign({ class: "execute-button px-1 py-0" }));
            var __VLS_9 = {}.OhVueIcon;
            /** @type {[typeof __VLS_components.OhVueIcon, ]} */ ;
            // @ts-ignore
            var __VLS_10 = __VLS_asFunctionalComponent(__VLS_9, new __VLS_9({
                name: "fa-play",
                scale: "1",
            }));
            var __VLS_11 = __VLS_10.apply(void 0, __spreadArray([{
                    name: "fa-play",
                    scale: "1",
                }], __VLS_functionalComponentArgsRest(__VLS_10), false));
        }
    }
    __VLS_asFunctionalElement(__VLS_intrinsicElements.div, __VLS_intrinsicElements.div)(__assign({ class: "d-flex flex-wrap gap-2 mb-2" }));
    for (var _e = 0, _f = __VLS_getVForSourceType((service.parameters)); _e < _f.length; _e++) {
        var param = _f[_e][0];
        __VLS_asFunctionalElement(__VLS_intrinsicElements.div, __VLS_intrinsicElements.div)(__assign({ key: (param.name) }, { class: "input-layer p-2 d-flex align-items-center flex-grow-1" }));
        __VLS_asFunctionalElement(__VLS_intrinsicElements.div, __VLS_intrinsicElements.div)(__assign({ class: (['param-name white-text-outline', { disabled: param.readOnly }]) }));
        (param.name);
        __VLS_asFunctionalElement(__VLS_intrinsicElements.div, __VLS_intrinsicElements.div)(__assign({ class: "vertical-rule mx-2" }));
        var __VLS_13 = ((__VLS_ctx.componentMap[param.inputDataType]));
        // @ts-ignore
        var __VLS_14 = __VLS_asFunctionalComponent(__VLS_13, new __VLS_13(__assign({ serviceParam: (param), modelValue: (__VLS_ctx.inputs[param.name]), isOutput: (false) }, { class: "flex-grow-1" })));
        var __VLS_15 = __VLS_14.apply(void 0, __spreadArray([__assign({ serviceParam: (param), modelValue: (__VLS_ctx.inputs[param.name]), isOutput: (false) }, { class: "flex-grow-1" })], __VLS_functionalComponentArgsRest(__VLS_14), false));
    }
    if ((_b = service.returns) === null || _b === void 0 ? void 0 : _b.length) {
        __VLS_asFunctionalElement(__VLS_intrinsicElements.div, __VLS_intrinsicElements.div)(__assign({ class: "d-flex flex-wrap gap-2" }));
        for (var _g = 0, _h = __VLS_getVForSourceType((service.returns)); _g < _h.length; _g++) {
            var returnInstance = _h[_g][0];
            __VLS_asFunctionalElement(__VLS_intrinsicElements.div, __VLS_intrinsicElements.div)(__assign({ key: (returnInstance.name) }, { class: "input-layer p-2 d-flex align-items-center flex-grow-1" }));
            __VLS_asFunctionalElement(__VLS_intrinsicElements.div, __VLS_intrinsicElements.div)(__assign({ class: "param-name white-text-outline disabled" }));
            (returnInstance.name);
            __VLS_asFunctionalElement(__VLS_intrinsicElements.div, __VLS_intrinsicElements.div)(__assign({ class: "vertical-rule mx-2" }));
            var __VLS_17 = ((__VLS_ctx.componentMap[returnInstance.outputDataType]));
            // @ts-ignore
            var __VLS_18 = __VLS_asFunctionalComponent(__VLS_17, new __VLS_17(__assign({ serviceParam: (returnInstance), modelValue: (__VLS_ctx.inputs[returnInstance.name]), isOutput: (true) }, { class: "flex-grow-1" })));
            var __VLS_19 = __VLS_18.apply(void 0, __spreadArray([__assign({ serviceParam: (returnInstance), modelValue: (__VLS_ctx.inputs[returnInstance.name]), isOutput: (true) }, { class: "flex-grow-1" })], __VLS_functionalComponentArgsRest(__VLS_18), false));
        }
    }
    if ((_c = service.returns) === null || _c === void 0 ? void 0 : _c.length) {
        __VLS_asFunctionalElement(__VLS_intrinsicElements.div, __VLS_intrinsicElements.div)(__assign({ class: "mt-auto text-end" }));
        __VLS_asFunctionalElement(__VLS_intrinsicElements.span, __VLS_intrinsicElements.span)(__assign({ class: "smallest-text bold-text" }));
        (__VLS_ctx.currentTime);
    }
    var __VLS_8;
}
var __VLS_3;
/** @type {__VLS_StyleScopedClasses['g-3']} */ ;
/** @type {__VLS_StyleScopedClasses['col-12']} */ ;
/** @type {__VLS_StyleScopedClasses['col-md-6']} */ ;
/** @type {__VLS_StyleScopedClasses['input-layer']} */ ;
/** @type {__VLS_StyleScopedClasses['p-2']} */ ;
/** @type {__VLS_StyleScopedClasses['h-100']} */ ;
/** @type {__VLS_StyleScopedClasses['d-flex']} */ ;
/** @type {__VLS_StyleScopedClasses['flex-column']} */ ;
/** @type {__VLS_StyleScopedClasses['d-flex']} */ ;
/** @type {__VLS_StyleScopedClasses['justify-content-between']} */ ;
/** @type {__VLS_StyleScopedClasses['align-items-center']} */ ;
/** @type {__VLS_StyleScopedClasses['mb-2']} */ ;
/** @type {__VLS_StyleScopedClasses['service-name']} */ ;
/** @type {__VLS_StyleScopedClasses['shadowy']} */ ;
/** @type {__VLS_StyleScopedClasses['flex-grow-1']} */ ;
/** @type {__VLS_StyleScopedClasses['mx-2']} */ ;
/** @type {__VLS_StyleScopedClasses['loader-bar']} */ ;
/** @type {__VLS_StyleScopedClasses['shrink']} */ ;
/** @type {__VLS_StyleScopedClasses['d-flex']} */ ;
/** @type {__VLS_StyleScopedClasses['align-items-center']} */ ;
/** @type {__VLS_StyleScopedClasses['execute-button']} */ ;
/** @type {__VLS_StyleScopedClasses['px-1']} */ ;
/** @type {__VLS_StyleScopedClasses['py-0']} */ ;
/** @type {__VLS_StyleScopedClasses['d-flex']} */ ;
/** @type {__VLS_StyleScopedClasses['flex-wrap']} */ ;
/** @type {__VLS_StyleScopedClasses['gap-2']} */ ;
/** @type {__VLS_StyleScopedClasses['mb-2']} */ ;
/** @type {__VLS_StyleScopedClasses['input-layer']} */ ;
/** @type {__VLS_StyleScopedClasses['p-2']} */ ;
/** @type {__VLS_StyleScopedClasses['d-flex']} */ ;
/** @type {__VLS_StyleScopedClasses['align-items-center']} */ ;
/** @type {__VLS_StyleScopedClasses['flex-grow-1']} */ ;
/** @type {__VLS_StyleScopedClasses['disabled']} */ ;
/** @type {__VLS_StyleScopedClasses['param-name']} */ ;
/** @type {__VLS_StyleScopedClasses['white-text-outline']} */ ;
/** @type {__VLS_StyleScopedClasses['vertical-rule']} */ ;
/** @type {__VLS_StyleScopedClasses['mx-2']} */ ;
/** @type {__VLS_StyleScopedClasses['flex-grow-1']} */ ;
/** @type {__VLS_StyleScopedClasses['d-flex']} */ ;
/** @type {__VLS_StyleScopedClasses['flex-wrap']} */ ;
/** @type {__VLS_StyleScopedClasses['gap-2']} */ ;
/** @type {__VLS_StyleScopedClasses['input-layer']} */ ;
/** @type {__VLS_StyleScopedClasses['p-2']} */ ;
/** @type {__VLS_StyleScopedClasses['d-flex']} */ ;
/** @type {__VLS_StyleScopedClasses['align-items-center']} */ ;
/** @type {__VLS_StyleScopedClasses['flex-grow-1']} */ ;
/** @type {__VLS_StyleScopedClasses['param-name']} */ ;
/** @type {__VLS_StyleScopedClasses['white-text-outline']} */ ;
/** @type {__VLS_StyleScopedClasses['disabled']} */ ;
/** @type {__VLS_StyleScopedClasses['vertical-rule']} */ ;
/** @type {__VLS_StyleScopedClasses['mx-2']} */ ;
/** @type {__VLS_StyleScopedClasses['flex-grow-1']} */ ;
/** @type {__VLS_StyleScopedClasses['mt-auto']} */ ;
/** @type {__VLS_StyleScopedClasses['text-end']} */ ;
/** @type {__VLS_StyleScopedClasses['smallest-text']} */ ;
/** @type {__VLS_StyleScopedClasses['bold-text']} */ ;
var __VLS_dollars;
var __VLS_self = (await Promise.resolve().then(function () { return require('vue'); })).defineComponent({
    setup: function () {
        return {
            MDBCol: mdb_vue_ui_kit_1.MDBCol,
            MDBRow: mdb_vue_ui_kit_1.MDBRow,
            OhVueIcon: oh_vue_icons_1.OhVueIcon,
            inputs: inputs,
            currentTime: currentTime,
            isShrunk: isShrunk,
            toggleLoader: toggleLoader,
            componentMap: componentMap,
        };
    },
    __typeProps: {},
});
exports.default = (await Promise.resolve().then(function () { return require('vue'); })).defineComponent({
    setup: function () {
        return {};
    },
    __typeProps: {},
});
; /* PartiallyEnd: #4569/main.vue */
