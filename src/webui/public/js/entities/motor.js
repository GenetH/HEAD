define(['application', 'lib/api', 'lib/utilities'], function (App, api, utilities) {
    App.module('Entities', function (Entities, App, Backbone, Marionette, $, _) {
        Entities.Motor = Backbone.Model.extend({
            initialize: function () {
                // not selected by default
                this.set('selected', false);

                // not selected by default
                this.set('selected', false);

                // send motor command when updating value
                this.on('change:value', this.checkUpdatedValue);
            },
            selected: function (selected) {
                if (typeof selected == 'undefined') {
                    // return status if arg undefined
                    return this.get('selected');
                } else {
                    // set status otherwise
                    this.set('selected', !!selected);
                }
            },
            checkUpdatedValue: function () {
                if (this.previous('value') != this.get('value'))
                    api.sendMotorCommand(this.toJSON(), this.get('value'));
            },
            getDegrees: function (attribute) {
                return Math.round(utilities.radToDeg(this.get(attribute))) || 0;
            },
            setDegrees: function (attribute, value, options) {
                return this.set(attribute, utilities.degToRad(value), options);
            },
            // Sets value 0-1 mapped to min and max.
            setRelativeVal: function (attribute, value) {
                var min = this.get('min');
                var max = this.get('max');
                var v = (min + (max - min) * value);
                this.set(attribute, v);
            },
            getRelativeVal: function (attribute) {
                var min = this.get('min');
                var max = this.get('max');
                var v = this.get(attribute);
                return Math.min(1, Math.max(0, (v - min) / (max - min)));
            }
        });
        Entities.MotorCollection = Backbone.Collection.extend({
            model: Entities.Motor,
            comparator: 'sort_no',
            sync: function (successCallback, errorCallback) {
                var data = _.filter(this.toJSON(), function (motor) {
                    delete motor['selected'];
                    delete motor['state'];
                    delete motor['editable'];
                    delete motor['value'];

                    return motor;
                });

                $.ajax("/motors/update/" + api.config.robot, {
                    data: JSON.stringify(data),
                    type: 'POST',
                    dataType: "json",
                    success: function (response) {
                        if (response.error) {
                            if (typeof errorCallback == 'function')
                                errorCallback(response.error);
                        } else if (typeof successCallback == 'function')
                            successCallback();
                    },
                    error: function () {
                        if (typeof errorCallback == 'function')
                            errorCallback();
                    }
                });
            },
            fetchFromParam: function (callback) {
                var self = this;

                api.getMotorsFromParam(function (motors) {
                    self.add(motors);

                    if (typeof callback == 'function')
                        callback(motors);
                });
            },
            fetchFromFile: function (callback) {
                var self = this;

                $.ajax('/motors/get/' + api.config.robot, {
                    dataType: 'json',
                    success: function (response) {
                        self.add(response.motors);

                        if (typeof callback == 'function')
                            callback(response.motors);
                    }
                });
            },
            setDefaultValues: function (silent) {
                this.each(function (motor) {
                    motor.set({value: motor.get('default')}, {silent: !!silent});
                })
            },
            getRelativePositions: function () {
                var positions = {};

                this.each(function (motor) {
                    if (motor.selected())
                        positions[motor.get('name')] = motor.getRelativeVal('value');
                });
                return positions;
            },
            fetchStates: function (success) {
                var self = this;

                api.getMotorStates(function (states) {
                    self.each(function (motor) {
                        var mi = states.motors.indexOf(motor.get('name'));
                        if (mi > -1) {
                            // prevent from going over extreme positions
                            var mv = states.angles[mi];
                            if (mv < motor.get('min')) mv = motor.get('min');
                            if (mv > motor.get('max')) mv = motor.get('max');
                            motor.set('value', mv);
                            motor.set('state', {
                                error: states.errors[mi],
                                load: states.loads[mi],
                                temperature: states.temperatures[mi]
                            });
                        }
                    });

                    // fire success callback
                    if (typeof success == 'function') success();
                });
            }
        });
    })
})
