import type { GeoForm } from "@/models/GeoForm";
type __VLS_Props = {
    form: GeoForm;
};
declare const _default: import("vue").DefineComponent<__VLS_Props, {}, {}, {}, {}, import("vue").ComponentOptionsMixin, import("vue").ComponentOptionsMixin, {} & {
    delete: (form: GeoForm) => any;
    locate: (form: GeoForm) => any;
    edit: (form: GeoForm) => any;
}, string, import("vue").PublicProps, Readonly<__VLS_Props> & Readonly<{
    onDelete?: (form: GeoForm) => any;
    onLocate?: (form: GeoForm) => any;
    onEdit?: (form: GeoForm) => any;
}>, {}, {}, {}, {}, string, import("vue").ComponentProvideOptions, false, {}, any>;
export default _default;
