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
var isLoading = (0, vue_1.ref)(true);
// Keep internal value in sync with parent
(0, vue_1.watch)(function () { return props.modelValue; }, function (newVal) {
    inputValue.value = newVal;
});
// Emit changes to parent
(0, vue_1.watch)(inputValue, function (newVal) {
    emit('update:modelValue', newVal);
});
debugger; /* PartiallyEnd: #3632/scriptSetup.vue */
var __VLS_ctx = {};
var __VLS_components;
var __VLS_directives;
// CSS variable injection 
// CSS variable injection end 
__VLS_asFunctionalElement(__VLS_intrinsicElements.div, __VLS_intrinsicElements.div)(__assign({ class: "bool-background-true h-100 w-100 d-flex align-items-center rounded-end" }));
__VLS_asFunctionalElement(__VLS_intrinsicElements.input)(__assign({ disabled: (props.serviceParam.readOnly), type: "checkbox" }, { class: "rounded m-2" }));
(__VLS_ctx.inputValue);
/** @type {__VLS_StyleScopedClasses['bool-background-true']} */ ;
/** @type {__VLS_StyleScopedClasses['h-100']} */ ;
/** @type {__VLS_StyleScopedClasses['w-100']} */ ;
/** @type {__VLS_StyleScopedClasses['d-flex']} */ ;
/** @type {__VLS_StyleScopedClasses['align-items-center']} */ ;
/** @type {__VLS_StyleScopedClasses['rounded-end']} */ ;
/** @type {__VLS_StyleScopedClasses['rounded']} */ ;
/** @type {__VLS_StyleScopedClasses['m-2']} */ ;
var __VLS_dollars;
var __VLS_self = (await Promise.resolve().then(function () { return require('vue'); })).defineComponent({
    setup: function () {
        return {
            inputValue: inputValue,
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
