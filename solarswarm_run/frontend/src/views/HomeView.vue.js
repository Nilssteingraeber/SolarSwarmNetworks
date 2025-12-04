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
var TopMenu_vue_1 = require("@/components/topMenu/TopMenu.vue");
var Cesium_vue_1 = require("./Cesium.vue");
var nodes_1 = require("@/stores/nodes");
var mdb_vue_ui_kit_1 = require("mdb-vue-ui-kit");
var nodesStore = (0, nodes_1.useNodesStore)();
debugger; /* PartiallyEnd: #3632/scriptSetup.vue */
var __VLS_ctx = {};
var __VLS_components;
var __VLS_directives;
/** @type {[typeof Cesium, ]} */ ;
// @ts-ignore
var __VLS_0 = __VLS_asFunctionalComponent(Cesium_vue_1.default, new Cesium_vue_1.default({}));
var __VLS_1 = __VLS_0.apply(void 0, __spreadArray([{}], __VLS_functionalComponentArgsRest(__VLS_0), false));
var __VLS_3 = {}.MDBRow;
/** @type {[typeof __VLS_components.MDBRow, typeof __VLS_components.MDBRow, ]} */ ;
// @ts-ignore
var __VLS_4 = __VLS_asFunctionalComponent(__VLS_3, new __VLS_3({}));
var __VLS_5 = __VLS_4.apply(void 0, __spreadArray([{}], __VLS_functionalComponentArgsRest(__VLS_4), false));
__VLS_6.slots.default;
var __VLS_7 = {}.MDBCol;
/** @type {[typeof __VLS_components.MDBCol, typeof __VLS_components.MDBCol, ]} */ ;
// @ts-ignore
var __VLS_8 = __VLS_asFunctionalComponent(__VLS_7, new __VLS_7({}));
var __VLS_9 = __VLS_8.apply(void 0, __spreadArray([{}], __VLS_functionalComponentArgsRest(__VLS_8), false));
__VLS_10.slots.default;
/** @type {[typeof TopMenu, ]} */ ;
// @ts-ignore
var __VLS_11 = __VLS_asFunctionalComponent(TopMenu_vue_1.default, new TopMenu_vue_1.default({}));
var __VLS_12 = __VLS_11.apply(void 0, __spreadArray([{}], __VLS_functionalComponentArgsRest(__VLS_11), false));
var __VLS_10;
var __VLS_6;
var __VLS_dollars;
var __VLS_self = (await Promise.resolve().then(function () { return require('vue'); })).defineComponent({
    setup: function () {
        return {
            TopMenu: TopMenu_vue_1.default,
            Cesium: Cesium_vue_1.default,
            MDBCol: mdb_vue_ui_kit_1.MDBCol,
            MDBRow: mdb_vue_ui_kit_1.MDBRow,
        };
    },
});
exports.default = (await Promise.resolve().then(function () { return require('vue'); })).defineComponent({
    setup: function () {
        return {};
    },
});
; /* PartiallyEnd: #4569/main.vue */
