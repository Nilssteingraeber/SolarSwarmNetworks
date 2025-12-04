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
Object.defineProperty(exports, "__esModule", { value: true });
var vue_1 = require("vue");
var props = (0, vue_1.defineProps)();
var emit = (0, vue_1.defineEmits)();
var inputValue = (0, vue_1.ref)(props.modelValue);
var inputRef = (0, vue_1.ref)(null);
var spanRef = (0, vue_1.ref)(null);
// keep internal value in sync with parent
(0, vue_1.watch)(function () { return props.modelValue; }, function (newVal) {
    inputValue.value = newVal;
});
(0, vue_1.watch)(inputValue, function (newVal) {
    emit('update:modelValue', newVal);
    resizeInput();
});
function resizeInput() {
    var _a;
    if (!inputRef.value || !spanRef.value)
        return;
    spanRef.value.textContent = (_a = inputValue.value) !== null && _a !== void 0 ? _a : '';
    var textWidth = spanRef.value.offsetWidth;
    // Add buffer: ~18px for spinner + ~8px padding
    inputRef.value.style.width = "".concat(textWidth + 26, "px");
}
(0, vue_1.onMounted)(function () { return resizeInput(); });
debugger; /* PartiallyEnd: #3632/scriptSetup.vue */
var __VLS_ctx = {};
var __VLS_components;
var __VLS_directives;
// CSS variable injection 
// CSS variable injection end 
__VLS_asFunctionalElement(__VLS_intrinsicElements.div, __VLS_intrinsicElements.div)(__assign({ class: "bool-background-true d-inline-flex align-items-center rounded-end" }));
__VLS_asFunctionalElement(__VLS_intrinsicElements.input)(__assign({ ref: "inputRef", disabled: (props.serviceParam.readOnly), type: "number" }, { class: "rounded m-2 small-number-input" }));
(__VLS_ctx.inputValue);
/** @type {typeof __VLS_ctx.inputRef} */ ;
__VLS_asFunctionalElement(__VLS_intrinsicElements.span, __VLS_intrinsicElements.span)(__assign({ ref: "spanRef" }, { class: "hidden-measurer" }));
/** @type {typeof __VLS_ctx.spanRef} */ ;
/** @type {__VLS_StyleScopedClasses['bool-background-true']} */ ;
/** @type {__VLS_StyleScopedClasses['d-inline-flex']} */ ;
/** @type {__VLS_StyleScopedClasses['align-items-center']} */ ;
/** @type {__VLS_StyleScopedClasses['rounded-end']} */ ;
/** @type {__VLS_StyleScopedClasses['rounded']} */ ;
/** @type {__VLS_StyleScopedClasses['m-2']} */ ;
/** @type {__VLS_StyleScopedClasses['small-number-input']} */ ;
/** @type {__VLS_StyleScopedClasses['hidden-measurer']} */ ;
var __VLS_dollars;
var __VLS_self = (await Promise.resolve().then(function () { return require('vue'); })).defineComponent({
    setup: function () {
        return {
            inputValue: inputValue,
            inputRef: inputRef,
            spanRef: spanRef,
        };
    },
    __typeEmits: {},
    __typeProps: {},
});
exports.default = (await Promise.resolve().then(function () { return require('vue'); })).defineComponent({
    setup: function () {
        return {};
    },
    __typeEmits: {},
    __typeProps: {},
});
; /* PartiallyEnd: #4569/main.vue */
