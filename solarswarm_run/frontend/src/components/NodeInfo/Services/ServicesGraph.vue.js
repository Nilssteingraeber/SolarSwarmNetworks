"use strict";
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
var ServiceFactory_vue_1 = require("./ServiceFactory.vue");
var MockServices_1 = require("@/mock/MockServices");
debugger; /* PartiallyEnd: #3632/scriptSetup.vue */
var __VLS_ctx = {};
var __VLS_components;
var __VLS_directives;
// CSS variable injection 
// CSS variable injection end 
/** @type {[typeof ServiceFactory, ]} */ ;
// @ts-ignore
var __VLS_0 = __VLS_asFunctionalComponent(ServiceFactory_vue_1.default, new ServiceFactory_vue_1.default({
    services: (__VLS_ctx.MockServices),
}));
var __VLS_1 = __VLS_0.apply(void 0, __spreadArray([{
        services: (__VLS_ctx.MockServices),
    }], __VLS_functionalComponentArgsRest(__VLS_0), false));
var __VLS_3 = {};
var __VLS_2;
var __VLS_dollars;
var __VLS_self = (await Promise.resolve().then(function () { return require('vue'); })).defineComponent({
    setup: function () {
        return {
            ServiceFactory: ServiceFactory_vue_1.default,
            MockServices: MockServices_1.MockServices,
        };
    },
});
exports.default = (await Promise.resolve().then(function () { return require('vue'); })).defineComponent({
    setup: function () {
        return {};
    },
});
; /* PartiallyEnd: #4569/main.vue */
