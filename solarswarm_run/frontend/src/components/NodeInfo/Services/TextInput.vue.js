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
Object.defineProperty(exports, "__esModule", { value: true });
var mdb_vue_ui_kit_1 = require("mdb-vue-ui-kit");
var vue_1 = require("vue");
var props = (0, vue_1.defineProps)();
var emit = (0, vue_1.defineEmits)();
var inputValue = (0, vue_1.ref)(props.modelValue);
// Keep internal value in sync with parent
(0, vue_1.watch)(function () { return props.modelValue; }, function (newVal) {
    inputValue.value = newVal;
});
// Emit changes to parent
(0, vue_1.watch)(inputValue, function (newVal) {
    emit('update:modelValue', newVal);
});
// Ref to the textarea
var textareaRef = (0, vue_1.ref)(null);
// Auto-resize function: horizontal first, then vertical
var autoResize = function () {
    var _a;
    if (!textareaRef.value)
        return;
    var textarea = textareaRef.value.querySelector('textarea');
    if (!textarea)
        return;
    // Reset height to auto to shrink if needed
    textarea.style.height = 'auto';
    // Set width up to max
    var parentWidth = ((_a = textarea.parentElement) === null || _a === void 0 ? void 0 : _a.offsetWidth) || 0;
    textarea.style.width = 'auto'; // grow naturally
    textarea.style.width = Math.min(textarea.scrollWidth, parentWidth) + 'px';
    // Once width is full, grow height
    if (textarea.offsetWidth >= parentWidth) {
        textarea.style.height = textarea.scrollHeight + 'px';
    }
};
debugger; /* PartiallyEnd: #3632/scriptSetup.vue */
var __VLS_ctx = {};
var __VLS_components;
var __VLS_directives;
// CSS variable injection 
// CSS variable injection end 
__VLS_asFunctionalElement(__VLS_intrinsicElements.div, __VLS_intrinsicElements.div)(__assign({ class: "m-2" }, { ref: "textareaRef" }));
/** @type {typeof __VLS_ctx.textareaRef} */ ;
var __VLS_0 = {}.MDBTextarea;
/** @type {[typeof __VLS_components.MDBTextarea, ]} */ ;
// @ts-ignore
var __VLS_1 = __VLS_asFunctionalComponent(__VLS_0, new __VLS_0(__assign(__assign(__assign({ 'onInput': {} }, { type: "textarea", modelValue: (__VLS_ctx.inputValue) }), { class: "rounded resizable-input" }), { disabled: (__VLS_ctx.isOutput), rows: "1" })));
var __VLS_2 = __VLS_1.apply(void 0, __spreadArray([__assign(__assign(__assign({ 'onInput': {} }, { type: "textarea", modelValue: (__VLS_ctx.inputValue) }), { class: "rounded resizable-input" }), { disabled: (__VLS_ctx.isOutput), rows: "1" })], __VLS_functionalComponentArgsRest(__VLS_1), false));
var __VLS_4;
var __VLS_5;
var __VLS_6;
var __VLS_7 = {
    onInput: (__VLS_ctx.autoResize)
};
var __VLS_3;
/** @type {__VLS_StyleScopedClasses['m-2']} */ ;
/** @type {__VLS_StyleScopedClasses['rounded']} */ ;
/** @type {__VLS_StyleScopedClasses['resizable-input']} */ ;
var __VLS_dollars;
var __VLS_self = (await Promise.resolve().then(function () { return require('vue'); })).defineComponent({
    setup: function () {
        return {
            MDBTextarea: mdb_vue_ui_kit_1.MDBTextarea,
            inputValue: inputValue,
            textareaRef: textareaRef,
            autoResize: autoResize,
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
